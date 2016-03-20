//-------------------------------------------------------------------
// Based on Adafruit's Sous_Viduino
// Adapted to work with an OLED screen
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

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
double setpoint;
double input = 0;
double output;
bool is_on = false;
bool is_on_selection = false;
int turn_on_delay;
int turn_off_delay;
unsigned long started_cooking_at = 0;
unsigned long turn_on_at;
unsigned long turn_off_at;

// Temperature history
#define HISTORY_LEN 60
byte short_term_history[HISTORY_LEN];
unsigned long last_short_term_update;
volatile long on_time = 0;

// pid tuning parameters
double kp;
double ki;
double kd;

// EEPROM addresses for persisted data
const int sp_address = 0;
const int kp_address = 8;
const int ki_address = 16;
const int kd_address = 24;

//Specify the links and initial tuning parameters
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// 60 second Time Proportional output window
unsigned int window_size = 60000; 
unsigned long window_start_time;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte a_tune_mode_remember = 2;

double auto_tune_step = 500;
double auto_tune_noise = 1;
unsigned int auto_tune_lookback = 20;

boolean is_tuning = false;

PID_ATune a_tune(&input, &output);

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
	if (is_on) {
		drive_output();
	}
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

	if (digitalRead(SELECT_PIN) == LOW) {
		start_auto_tune();
	}

	// Start up the DS18B20 One Wire Temperature Sensor
	sensors.begin();
	sensors.getAddress(temp_sensor, 0);
	sensors.setResolution(temp_sensor, 12);
	sensors.setWaitForConversion(false);

	u8g.setFont(u8g_font_6x10);
	u8g.setFontPosTop();

	// Initialize the PID and related variables
	load_parameters();

	pid.SetTunings(kp, ki, kd);

	pid.SetSampleTime(1000);
	pid.SetOutputLimits(0, window_size);
	pid.SetMode(AUTOMATIC);

	Timer1.initialize(15000); // 15ms timer
	Timer1.attachInterrupt(timer_isr); // attach the service routine here
}

void update_history() {
	unsigned long now = millis();
	if (now - last_short_term_update > 1000) {
		for (byte i=1; i < HISTORY_LEN; i++) {
			short_term_history[i-1] = short_term_history[i];
		}
		short_term_history[HISTORY_LEN-1] = input;
		last_short_term_update = now;
	}

	// For the long-term history, we'll use the average temperature of the last minute
	// if (now - last_long_term_update > 3600000) {
	// 	double short_term_avg = short_term_history[0]/60.0;
	// 	for (byte i=1; i<60; i++) {
	// 		short_term_avg += short_term_history[i]/60.0;
	// 		long_term_history[i-1] = long_term_history[i];
	// 	}
	// 	long_term_history[59] = short_term_avg;
	// 	last_long_term_update = now;
	// }
}

void draw_history(byte *history) {
	// Draw horizontal dotted line for target temperature
	byte y_pos_setpoint = 73 - round(setpoint / 2.0);
	for (byte x=0; x < HISTORY_LEN; x+=2) {
		u8g.drawPixel(x + 35, y_pos_setpoint);
	}

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
	u8g.firstPage();
	do {
		if (digitalRead(BYPASS_PIN) == LOW) {
			// Bypass logic - If bypass is on, nothing else happens
			if (digitalRead(BYPASS_PIN) == LOW) {
				digitalWrite(RELAY_PIN, LOW);

				u8g.setScale2x2(); 
				main_screen->write("OFF", 32, 12, CENTER);
				u8g.undoScale();
				return;
			}
		} else if (millis() > ui.last_activity() + 2000) {
			// Show status screen
			main_screen->write(String(int(round(input))) + " deg", 64, 0, CENTER);
			if (is_tuning) {
				main_screen->write(String(kp) + " " + String(ki) + " " + String(kd), 64, 10, CENTER);
			}
			draw_history(short_term_history);
		} else {
			ui.update();
		}
	} while(u8g.nextPage());

	// Read updated values
	if (sensors.isConversionAvailable(0)) {
		input = sensors.getTempC(temp_sensor);
		sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
	}

	setpoint = temperature_range->get_value();
	turn_on_delay = turn_on_delay_range->get_value();
	turn_off_delay = turn_off_delay_range->get_value();

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
			digitalWrite(RELAY_PIN, HIGH);
		}
	}

	// As long as we're waiting for an on-delay to expire (is_on is selected but not yet set)
	// check if the cooking should start yet
	if(is_on_selection && !is_on) {
		if(millis() > turn_on_at) {
			on_toggle->name = "On";
			is_on = true;
			started_cooking_at = millis();
		} else {
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
	}

	update_history();

	pid.SetTunings(kp, ki, kd);
	do_pid_control();
}

void do_pid_control() {
	if (is_tuning) {
		// run the auto-tuner
		if (a_tune.Runtime()) { // returns 'true' when done
			finish_auto_tune();
		}
	} else {
		pid.Compute();
	}

	// Time Proportional relay state is updated regularly via timer interrupt.
	on_time = output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void drive_output() {  
	long now = millis();
	// Set the output
	// "on time" is proportional to the PID output
	if (now - window_start_time > window_size) { //time to shift the Relay Window
		window_start_time += window_size;
	}
	digitalWrite(RELAY_PIN, !((on_time > 100) && (on_time > (now - window_start_time))));
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************
void start_auto_tune() {
	// Remember the mode we were in
	a_tune_mode_remember = pid.GetMode();

	// set up the auto-tune parameters
	a_tune.SetNoiseBand(auto_tune_noise);
	a_tune.SetOutputStep(auto_tune_step);
	a_tune.SetLookbackSec((int)auto_tune_lookback);
	is_tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void finish_auto_tune() {
	is_tuning = false;

	// Extract the auto-tune calculated parameters
	kp = a_tune.GetKp();
	ki = a_tune.GetKi();
	kd = a_tune.GetKd();

	// Re-tune the PID and revert to normal control mode
	pid.SetTunings(kp,ki,kd);
	pid.SetMode(a_tune_mode_remember);

	// Persist any changed parameters to EEPROM
	save_parameters();
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void save_parameters() {
	eeprom_write_double(kp_address, kp);
	eeprom_write_double(ki_address, ki);
	eeprom_write_double(kd_address, kd);
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void load_parameters() {
	// Load from EEPROM
	kp = eeprom_read_double(kp_address);
	ki = eeprom_read_double(ki_address);
	kd = eeprom_read_double(kd_address);

	// Use defaults if EEPROM values are invalid
	if (isnan(kp)) {
		kp = 850;
		ki = 0.5;
		kd = 0.1;
	}  
}

void eeprom_write_double(int address, double value) {
	// Write floating point values to EEPROM
	byte* p = (byte*)(void*)&value;
	for (int i = 0; i < sizeof(value); i++) {
		EEPROM.write(address++, *p++);
	}
}

double eeprom_read_double(int address) {
	// Read floating point values from EEPROM
	double value = 0.0;
	byte* p = (byte*)(void*)&value;
	for (int i = 0; i < sizeof(value); i++) {
		*p++ = EEPROM.read(address++);
	}
	return value;
}