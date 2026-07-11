/* Minimal SSD1306 128x32 OLED driver (I2C, framebuffer based).
 * Text rendering uses a 5x7 font with integer scaling:
 * scale 1 -> 6x8 px cell, scale 2 -> 12x16 px cell.
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>

/* Returns false if the display did not acknowledge (e.g. unpowered). */
bool ssd1306_init(void);

/* Sends the 0xAE display-off command; failures are ignored. */
void ssd1306_display_off(void);

/* Clears the local framebuffer (does not transmit). */
void ssd1306_clear(void);

/* Draws text into the framebuffer at pixel position (x, y). */
void ssd1306_draw_text(int x, int y, const char *str, int scale);

/* Pixel width of a string at the given scale. */
int ssd1306_text_width(const char *str, int scale);

/* Fills (or clears) a rectangle in the framebuffer. */
void ssd1306_fill_rect(int x, int y, int w, int h, bool on);

/* Transmits the framebuffer to the display. */
bool ssd1306_update(void);

#endif /* SSD1306_H */
