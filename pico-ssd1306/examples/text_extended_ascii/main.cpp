// Define SSD1306_ASCII_FULL to use full ascii range (32 - 255)
#define SSD1306_ASCII_FULL

#include "pico/stdlib.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "hardware/i2c.h"

// Use the namespace for convenience
using namespace pico_ssd1306;

int main(){
    // Init i2c0 controller
    i2c_init(i2c0, 1000000);
    // Set up pins 12 and 13
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);

    // If you don't do anything before initializing a display pi pico is too fast and starts sending
    // commands before the screen controller had time to set itself up, so we add an artificial delay for
    // ssd1306 to set itself up
    sleep_ms(250);

    // Create a new display object at address 0x3D and size of 128x64
    SSD1306 display = SSD1306(i2c0, 0x3D, Size::W128xH64);

    // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
    // If your screen is upside down try setting it to 1 or 0
    display.setOrientation(0);

    // Draw text on display
    // After passing a pointer to display, we need to tell the function what font and text to use
    // Available fonts are listed in textRenderer's readme
    // Last we tell this function where to anchor the text
    // Anchor means top left of what we draw
    // We use \x escape to use chars by their hex numeration according to the ASCII table
    drawText(&display, font_5x8, "\x24 \xba \xb2", 0 ,0);
    drawText(&display, font_8x8, "\x24 \xba \xb2", 0 ,10);
    drawText(&display, font_12x16, "\x24 \xba \xb2", 0 ,20);
    drawText(&display, font_16x32, "\x24 \xba \xb2", 0 ,36);

    // Send buffer to the display
    display.sendBuffer();
}
