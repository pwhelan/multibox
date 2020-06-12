#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);

void setup()
{
	for (int i = 0; i < 5; i++) {
		pinMode(i+2, INPUT_PULLUP);
	}
	
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(0x76);

	const uint64_t pipe = 0xE0E0F1F1E0LL;
	radio.openWritingPipe(pipe);
	radio.enableDynamicPayloads();

	radio.powerUp();
}

void loop()
{
	char buf[5];

	memset(buf, 0, sizeof(buf));
	for (int i = 0; i < 5; i++) {
		if (digitalRead(i+2) == 0) {
			buf[i] = 0x01;
		}
	}
	radio.write(&buf, 5);
}
