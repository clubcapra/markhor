
#include <iostream>
// for delay function.
#include <chrono>
#include <thread>
#include <map>
#include <string>

// for signal handling
#include <signal.h>

#include <JetsonGPIO.h>

// using namespace GPIO;
 

// using namespace std;
// const map<string, int> output_pins{{"JETSON_XAVIER", 18}, {"JETSON_NANO", 33}};

// int get_output_pin()
// {
// 	if (output_pins.find(GPIO::model) == output_pins.end())
// 	{
// 		cerr << "PWM not supported on this board\n";
// 		terminate();
// 	}

// 	return output_pins.at(GPIO::model);
// }

// inline void delay(int s)
// {
// 	this_thread::sleep_for(chrono::seconds(s));
// }

// static bool end_this_program = false;

// void signalHandler(int s)
// {
// 	end_this_program = true;
// }



int main()
{
// 	// Pin Definitions
// 	int output_pin = get_output_pin();

// 	// When CTRL+C pressed, signalHandler will be called
// 	signal(SIGINT, signalHandler);

// 	// Pin Setup.
// 	// Board pin-numbering scheme
	// GPIO::setmode(GPIO::BOARD);
    printf("%d",GPIO::HIGH);

// 	// set pin as an output pin with optional initial state of HIGH
// 	GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
// 	GPIO::PWM p(output_pin, 50);
// 	auto val = 25.0;
// 	p.start(val);

// 	cout << "PWM running. Press CTRL+C to exit." << endl;

// 	while (!end_this_program)
// 	{
// 		delay(1);
// 	}

// 	p.stop();
// 	GPIO::cleanup();

	return 0;
}