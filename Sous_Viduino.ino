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

// ************************************************
// PID Variables and constants
// ************************************************

// Define Variables we'll be connecting to
double target_temperature;
double current_temperature = 0;

bool is_on = false;
bool is_on_selection = false;
bool heat_active = false;
int turn_on_delay;
int turn_off_delay;
unsigned long started_cooking_at = 0;
unsigned long turn_on_at;
unsigned long turn_off_at;

// Temperature history
#define HISTORY_LEN 60
#define HISTORY_INTERVAL 10000
byte history[HISTORY_LEN];
unsigned long last_history_update;
volatile long on_time = 0;

// ************************************************
// Display / UI
// ************************************************
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST|U8G_I2C_OPT_NO_ACK);

ToggleItem *on_toggle = new ToggleItem("On");
RangeItem *turn_on_delay_range = new RangeItem("On After", 0, 1440, 0, 30);
RangeItem *turn_off_delay_range = new RangeItem("Off After", 0, 2880, 0, 5);
RangeItem *temperature_range = new RangeItem("Temp", 0, 100, 58, 1);

Item *items[] = {
	on_toggle,
	turn_on_delay_range,
	turn_off_delay_range,
	temperature_range
};

Screen *main_screen = new Screen(u8g, items, 4);
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
		history[HISTORY_LEN-1] = int(round(current_temperature));
		last_history_update = now;
	}
}

void draw_history() {
	main_screen->write(String(current_temperature) + " deg", 64, 0, CENTER);

	// Draw horizontal dotted line for target temperature
	byte y_pos_target_temperature = 73 - round(target_temperature / 2.0);
	for (byte x=0; x < HISTORY_LEN; x+=2) {
		u8g.drawPixel(x + 35, y_pos_target_temperature);
	}
	main_screen->write(String(int(round(target_temperature))), 64, y_pos_target_temperature - 10, CENTER);

	// Draw history lines, one line per entry
	for (byte x=0; x < HISTORY_LEN; x++) {
		u8g.drawLine(x + 34, 63, x + 34, 73 - round(history[x] / 2.0));
	}

	byte y_pos_left = min(73 - round(history[0] / 2.0) - 4, 56);
	byte y_pos_right = min(73 - round(history[59] / 2.0) - 4, 56);
	main_screen->write(String(int(round(history[0]))), 33, y_pos_left, RIGHT);
	main_screen->write(String(int(round(history[59]))), 96, y_pos_right, LEFT);
}

void loop(){
	// Read updated values
	if (sensors.isConversionAvailable(0)) {
		current_temperature = sensors.getTempC(temp_sensor);
		sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
	}

	target_temperature = temperature_range->get_value();
	turn_on_delay = turn_on_delay_range->get_value();
	turn_off_delay = turn_off_delay_range->get_value();

	u8g.firstPage();
	do {
		if (digitalRead(BYPASS_PIN) == LOW) {
			// Bypass logic - If bypass is on, nothing else happens
			digitalWrite(RELAY_PIN, LOW);

			main_screen->write(String(int(round(current_temperature))) + " deg", 64, 0, CENTER);
			u8g.setScale2x2(); 
			main_screen->write("OFF", 32, 12, CENTER);
			u8g.undoScale();
			return;
		} else if (millis() > ui.last_activity() + 2000) {
			// Show status screen
			draw_history();
		} else {
			ui.update();
		}
	} while(u8g.nextPage());

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

		// Turn the relay on if it needs to be 
		// We use a simple hysteresis to avoid too much turning on or off
		if (current_temperature < (target_temperature - 0.5)) {
			digitalWrite(RELAY_PIN, LOW);
		} else if (current_temperature >= target_temperature) {
			digitalWrite(RELAY_PIN, HIGH);
		}
	} else {
		// Turn the relay off
		digitalWrite(RELAY_PIN, HIGH);
	}

	update_history();
}