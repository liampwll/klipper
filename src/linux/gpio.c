// Very basic support via a Linux gpiod device
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <sys/mman.h>
#include <fcntl.h> // open
#include <stdio.h> // snprintf
#include <stdlib.h> // atexit
#include <string.h> // memset
#include <sys/ioctl.h> // ioctl
#include <unistd.h> // close
#include "command.h" // shutdown
#include "gpio.h" // gpio_out_write
#include "internal.h" // report_errno
#include "sched.h" // sched_shutdown

#define GPIO_CONSUMER "klipper"

struct gpio_line {
    int offset;
    int state;
};
static struct gpio_line gpio_lines[MAX_GPIO_LINES];

#define GPIO_REG_MAP            0xFF634000
#define GPIOX_FSEL_REG_OFFSET   0x116
#define GPIOX_OUTP_REG_OFFSET   0x117
#define GPIOX_INP_REG_OFFSET    0x118
#define BLOCK_SIZE              (4*1024)

static volatile uint32_t *gpio;

// https://wiki.odroid.com/odroid-c4/application_note/gpio/memory_mapped_gpio
int
gpio_setup(void)
{
    int fd;

    if ((fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
        printf("Unable to open /dev/gpiomem\n");
        return -1;
    }

    gpio = mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_REG_MAP);
    if (gpio < 0) {
        printf("Mmap failed.\n");
        return -1;
    }

    return 0;
}

struct gpio_out
gpio_out_setup(uint32_t pin, uint8_t val)
{
    // % just to make sure we don't touch anything we shouldn't when testing.
    struct gpio_line *line = &gpio_lines[pin % MAX_GPIO_LINES];
    line->offset = pin % MAX_GPIO_LINES;
    struct gpio_out g = { .line = line };
    gpio_out_reset(g, val);
    return g;
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
    *(gpio + (GPIOX_FSEL_REG_OFFSET)) &= ~(1 << g.line->offset);
    gpio_out_write(g, val);
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
    if (val) {
        *(gpio + (GPIOX_OUTP_REG_OFFSET)) |= (1 << g.line->offset);
    } else {
        *(gpio + (GPIOX_OUTP_REG_OFFSET)) &= ~(1 << g.line->offset);
    }
    g.line->state = !!val;
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    gpio_out_write(g, !g.line->state);
}

void
gpio_out_toggle(struct gpio_out g)
{
    gpio_out_toggle_noirq(g);
}

struct gpio_in
gpio_in_setup(uint32_t pin, int8_t pull_up)
{
    struct gpio_line *line = &gpio_lines[pin];
    line->offset = pin;
    struct gpio_in g = { .line = line };
    gpio_in_reset(g, pull_up);
    return g;
}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
    // TODO: Check how pullups work on this board.
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    return (*(gpio + (GPIOX_INP_REG_OFFSET)) >> g.line->offset) & 1;
}
