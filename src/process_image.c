#include <ch.h>
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <stdio.h>
#include <camera/po8030.h>
#include <spi_comm.h>
#include "epuck1x/Asercom2.h"

#include "main.h"
#include "process_image.h"
#include "pi_regulator.h"
#include "ir_sensors.h"

static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE / 2; // middle (160 pixels wide)
bool config_cam_pid = false;

// semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer)
{
    uint16_t i = 0, begin = 0, end = 0, width = 0;
    uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
    uint64_t mean = 0; // attention changement

    uint16_t last_width = PXTOCM / get_goal_distance();

    // performs an average
    for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++)
    {
        mean += buffer[i];
    }
    mean /= IMAGE_BUFFER_SIZE;

    do
    {
        wrong_line = 0;
        // search for a begin
        while (stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
        {
            // the slope must at least be WIDTH_SLOPE wide and is compared
            // to the mean of the image
            if (buffer[i] > mean && buffer[i + WIDTH_SLOPE] < mean)
            {
                begin = i;
                stop = 1;
            }
            i++;
        }
        // if a begin was found, search for an end
        if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
        {
            stop = 0;

            while (stop == 0 && i < IMAGE_BUFFER_SIZE)
            {
                if (buffer[i] > mean && buffer[i - WIDTH_SLOPE] < mean)
                {
                    end = i;
                    stop = 1;
                }
                i++;
            }
            // if an end was not found
            if (i > IMAGE_BUFFER_SIZE || !end)
            {
                line_not_found = 1;
            }
        }
        else // if no begin was found
        {
            line_not_found = 1;
        }

        // if a line too small has been detected, continues the search
        if (!line_not_found && (end - begin) < MIN_LINE_WIDTH)
        {
            i = end;
            begin = 0;
            end = 0;
            stop = 0;
            wrong_line = 1;
        }
    } while (wrong_line);

    if (line_not_found)
    {
        begin = 0;
        end = 0;
        width = last_width;
    }
    else
    {
        last_width = width = (end - begin);
        line_position = (begin + end) / 2; // gives the line position.
    }

    // sets a maximum width or returns the measured width
    if ((PXTOCM / width) > MAX_DISTANCE)
    {
        return PXTOCM / MAX_DISTANCE;
    }
    else
    {
        return width;
    }
}

static THD_WORKING_AREA(waCaptureImage, 2048);
static THD_FUNCTION(CaptureImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    // Configure camera with resolution of 160x120 and no subsampling (1)
    // cam_advanced_config(PO8030_FORMAT_RGB565, 0, 0, 160, 120, SUBSAMPLING_X1, SUBSAMPLING_X1);

    while (1)
    {
        if (get_mode_one_on() == 1 && get_object_found() == true)
        {
            // starts a capture
            dcmi_capture_start();
            // waits for the capture to be done
            wait_image_ready();
            // signals an image has been captured
            chBSemSignal(&image_ready_sem);
            // chThdSleepMilliseconds(100);
        }
        else
        {
            chThdSleepMilliseconds(100);
        }
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t *img_buff_ptr;
    uint8_t image[160 * 2] = {0}; // Buffer size adjusted to 160*2 (two lines of the image)
    uint16_t lineWidth = 0;

    while (1)
    {
        if (get_mode_one_on() == 1 && get_object_found() == true)
        {
            // waits until an image has been captured
            chBSemWait(&image_ready_sem);
            // gets the pointer to the array filled with the last image in RGB565
            img_buff_ptr = dcmi_get_last_image_ptr();

            // Extracts only the red pixels from the USED_LINE and USED_LINE+1
            // Assuming that USED_LINE is a constant representing the line number (e.g., 60)
            // Image buffer will be fetched for the two lines: USED_LINE and USED_LINE+1
            uint16_t start_index = USED_LINE * 320;     // Start of USED_LINE in the buffer
            uint16_t end_index = (USED_LINE + 1) * 320; // Start of USED_LINE + 1 in the buffer

            // Copy the two lines (USED_LINE and USED_LINE+1)
            for (uint16_t i = start_index; i < end_index; i += 2) // 320 bytes per line for RGB565 format
            {
                image[(i - start_index) / 2] = (uint8_t)img_buff_ptr[i] & 0xF8; // Red pixel extraction
            }

            // search for a line in the image and get its width in pixels
            lineWidth = extract_line_width(image);

            // converts the width into a distance between the robot and the camera
            if (lineWidth)
            {
                distance_cm = PXTOCM / lineWidth;
            }
            // chThdSleepMilliseconds(100);
        }
        else
        {
            chThdSleepMilliseconds(100);
        }
    }
}

float get_distance_cm(void)
{
    return distance_cm;
}

uint16_t get_line_position(void)
{
    return line_position;
}

void process_image_start(void)
{
    chThdCreateStatic(waProcessImage,
                      sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage,
                      sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

bool get_config_cam_pid(void)
{
    return config_cam_pid;
}

void set_config_cam_pid(bool val)
{
    config_cam_pid = val;
    return;
}
