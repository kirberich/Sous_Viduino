//-------------------------------------------------------------------
// Based on Adafruit's Sous_Viduino
// Adapted to work with an OLED screen
//------------------------------------------------------------------

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// Display
#include <U8glib.h>
#include <u8g_ui.h>

// Timer
#include <TimerOne.h>

// ************************************************
// Pin definitions
// ************************************************

// Button pins
#define UP_PIN 10
#define DOWN_PIN 11
#define SELECT_PIN 12
#define BYPASS_PIN 13

// Output Relay
#define RELAY_PIN 2

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)

#define ONE_WIRE_GND 5
#define ONE_WIRE_PWR 6
#define ONE_WIRE_BUS 7

#define RISING 0
#define FALLING 1
#define STEADY 2

// ************************************************
// PID Variables and constants
// ************************************************

// Define Variables we'll be connecting to
double target_temperature;
double current_temperature = 0;

bool is_on = false;
bool is_on_selection = false;
bool heat_active = false;
bool heat_active_requested = false;
unsigned long heat_last_changed;
int turn_on_delay;
int turn_off_delay;
unsigned long started_cooking_at = 0;
unsigned long turn_on_at;
unsigned long turn_off_at;

// Temperature history
#define HISTORY_LEN 90
#define HISTORY_INTERVAL 3000
double history[HISTORY_LEN];
double current_avg;
unsigned long last_history_update;
unsigned long last_avg_update;
volatile long on_time = 0;
byte temp_direction = STEADY;

String direction_strings[] = {
	"Rising",
	"Falling",
	"Steady"
};

// ************************************************
// Display / UI
// ************************************************
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST|U8G_I2C_OPT_NO_ACK);

ToggleItem *on_toggle = new ToggleItem("On");
RangeItem *turn_on_delay_range = new RangeItem("On After", 0, 1440, 0, 30);
RangeItem *turn_off_delay_range = new RangeItem("Off After", 0, 2880, 0, 5);
RangeItem *temperature_range = new RangeItem("Temp", 0, 100, 58, 1);
RangeItem *overshoot_range = new RangeItem("Overshoot", 0, 10, 2, 0.1);
RangeItem *lookback_time = new RangeItem("Lookback", 0, 120, 30, 10);

Item *items[] = {
	on_toggle,
	turn_on_delay_range,
	turn_off_delay_range,
	temperature_range,
	overshoot_range,
	lookback_time,
};

Screen *main_screen = new Screen(u8g, items, 6);
U8G_UI ui = U8G_UI(main_screen, UP_PIN, DOWN_PIN, SELECT_PIN);

// ************************************************
// Sensor Variables and constants
// Setup a OneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// ************************************************
OneWire one_wire(ONE_WIRE_BUS);

// Pass our one_wire reference to Dallas Temperature. 
DallasTemperature sensors(&one_wire);

// arrays to hold device address
DeviceAddress temp_sensor;

void timer_isr() {
	// Timer Interrupt Handler
	ui.handle_input();
}

void setup() {
	// Initialize Relay Control:
	pinMode(RELAY_PIN, OUTPUT);    // output mode to drive relay
	digitalWrite(RELAY_PIN, HIGH); // Turn relay off to start

	// Set up Ground & Power for the sensor from GPIO pins
	pinMode(ONE_WIRE_GND, OUTPUT);
	digitalWrite(ONE_WIRE_GND, LOW);

	pinMode(ONE_WIRE_PWR, OUTPUT);
	digitalWrite(ONE_WIRE_PWR, HIGH);

	pinMode(BYPASS_PIN, INPUT);
	digitalWrite(BYPASS_PIN, HIGH);

	// Start up the DS18B20 One Wire Temperature Sensor
	sensors.begin();
	sensors.getAddress(temp_sensor, 0);
	sensors.setResolution(temp_sensor, 12);
	sensors.setWaitForConversion(false);

	u8g.setFont(u8g_font_6x10);
	u8g.setFontPosTop();

	Timer1.initialize(15000); // 15ms timer
	Timer1.attachInterrupt(timer_isr); // attach the service routine here
}

void update_history() {
	unsigned long now = millis();
	if (now - last_history_update > HISTORY_INTERVAL) {
		for (byte i=1; i < HISTORY_LEN; i++) {
			history[i-1] = history[i];
		}
		history[HISTORY_LEN-1] = current_avg;
		last_history_update = now;
	}
}

double get_history_avg() {
	double history_sum;
	for (byte i=0; i < HISTORY_LEN; i++) {
		history_sum += history[i];
	}
	return history_sum/HISTORY_LEN;
}

double temperature_seconds_ago(double seconds) {
	double history_steps_ago = min(HISTORY_LEN - 2, int(round((1000*seconds)/HISTORY_INTERVAL)));
	double between_frac = history_steps_ago - int(history_steps_ago);
	int history_newer_index = HISTORY_LEN - 1 - int(history_steps_ago);
	double history_newer = history[history_newer_index];
	double history_older = history[history_newer_index - 1];

	return history_older * between_frac + history_newer * (1.0 - between_frac);
}

void draw_history() {
	main_screen->write(String(current_avg) + " deg", 64, 0, CENTER);
	main_screen->write(direction_strings[temp_direction], 64, 10, CENTER);
	// Draw horizontal dotted line for target temperature
	byte y_pos_target_temperature = 73 - round(target_temperature / 2.0);
	byte left_edge = 64 - HISTORY_LEN/2;
	for (byte x=0; x < HISTORY_LEN; x+=2) {
		u8g.drawPixel(x + left_edge, y_pos_target_temperature);
	}
	main_screen->write(String(int(round(target_temperature))), 64, y_pos_target_temperature - 10, CENTER);

	// Draw history lines, one line per entry
	for (byte x=0; x < HISTORY_LEN; x++) {
		u8g.drawLine(x + left_edge, 63, x + left_edge, 73 - round(history[x] / 2.0));
	}

	byte y_pos_left = min(73 - round(history[0] / 2.0) - 4, 56);
	byte y_pos_right = min(73 - round(history[HISTORY_LEN - 1] / 2.0) - 4, 56);
	main_screen->write(String(int(round(history[0]))), left_edge - 2, y_pos_left, RIGHT);
	main_screen->write(String(int(round(history[HISTORY_LEN - 1]))), left_edge + HISTORY_LEN + 2, y_pos_right, LEFT);
}

void loop(){
	// Read updated values
	if (sensors.isConversionAvailable(0)) {
		current_temperature = sensors.getTempC(temp_sensor);
		sensors.requestTemperatures(); // prime the pump for the next one - but don't wait

		if(!current_avg) {
			current_avg = current_temperature;
		}

		// Update the rolling average of current temperatures. This is recorded in a window of the same size as one 
		// history entry - so everytime the history is updated, it'll be updated with the average temperature since the last history update.
		unsigned long avg_frame_time = millis() - last_avg_update;
		if (avg_frame_time > 100) {
			last_avg_update = millis();
			double avg_fraction = min(1.0, double(avg_frame_time) / HISTORY_INTERVAL);

			current_avg = (1 - avg_fraction) * current_avg + avg_fraction * current_temperature;
		}
	}

	// Set relay state
	if (heat_active != heat_active_requested && millis() > heat_last_changed + 5000) {
		heat_last_changed = millis();
		digitalWrite(RELAY_PIN, !heat_active_requested);
		heat_active = heat_active_requested;
	}
	digitalWrite(RELAY_PIN, !heat_active);

	target_temperature = temperature_range->get_value();
	turn_on_delay = turn_on_delay_range->get_value();
	turn_off_delay = turn_off_delay_range->get_value();

	u8g.firstPage();
	do {
		if (digitalRead(BYPASS_PIN) == LOW) {
			// Bypass logic - If bypass is on, nothing else happens
			heat_active_requested = true;

			main_screen->write(String(int(round(current_temperature))) + " deg", 64, 0, CENTER);
			u8g.setScale2x2(); 
			main_screen->write("OFF", 32, 12, CENTER);
			u8g.undoScale();
		} else if (millis() > ui.last_activity() + 2000) {
			// Show status screen
			draw_history();
		} else {
			ui.update();
		}
	} while(u8g.nextPage());

	if (digitalRead(BYPASS_PIN) == LOW) {
		return;
	}

	// When the on state changes in the menu, turn the cooking off or set the on-delay
	if (on_toggle->get_value() != is_on_selection) {
		is_on_selection = on_toggle->get_value();

		if (is_on_selection) {
			// When on is selected, set the time that the cooking should be started at
			turn_on_at = millis() + turn_on_delay * 60000;
		} else {
			// When off is selected, turn off cooking immediately.
			is_on = false;
			on_toggle->name = "On";
		}
	}

	// As long as we're waiting for an on-delay to expire (is_on is selected but not yet set)
	// check if the cooking should start yet
	if(is_on_selection && !is_on) {
		if(millis() > turn_on_at) {
			on_toggle->name = "On";
			is_on = true;
			started_cooking_at = millis();
		} else if (turn_on_delay) {
			on_toggle->name = "On (in " + String(round((turn_on_at - millis())/60000.0)) + " min)";
		}
	}

	// Update the turn-off delay - it's relative to the cooking-start time, so we can just update it whenever
	if (is_on) {
		turn_off_at = started_cooking_at + turn_off_delay * 60000;

		if (turn_off_delay) {
			on_toggle->name = "On (for " + String(round((turn_off_at - millis())/60000.0)) + " min)";
			if (millis() > turn_off_at) {
				on_toggle->set_value(false);
			}
		}

		double temp_old = temperature_seconds_ago(lookback_time->get_value());
		if(current_avg > temp_old + 0.1) {
			temp_direction = RISING;
		} else if (current_avg < temp_old - 0.1) {
			temp_direction = FALLING;
		} else {
			temp_direction = STEADY;
		}

		// Turn the relay on if it needs to be.
		if (current_avg < (target_temperature - overshoot_range->get_value())) {
			// The temperature is low - turn on the heat.
			heat_active_requested = true;
		} else if (current_avg > target_temperature) {
			// The temperature is higher than the target - turn off the heat.
			heat_active_requested = false;
		} else if (temp_direction == RISING) {
			// Temperature is rising and we're approaching the target, cut off now to account for overshoot.
			heat_active_requested = false;
		} else if (temp_direction == FALLING) {
			// The temperature is falling and we're lower than the target, turn on the heat.
			heat_active_requested = true;
		} else {
			// This case only gets called if the temperature is reasonable steady and it's below the target
			if (current_avg < target_temperature - 0.5) {
				heat_active_requested = true;
			} else {
				// Do some fine-tuning. Turn the relay on if the current temperature is even slightly less than a recent temperature
				if (current_avg < temp_old) {
					// Super lazy 50% duty cycle hackery.
					if (millis() - heat_last_changed > 5000) {
						heat_active_requested = !heat_active_requested;
					}
				} else {
					heat_active_requested = false;
				}
			}
		}
	} else {
		// Turn the relay off
		heat_active_requested = false;
	}

	update_history();
}