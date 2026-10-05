SOFTWARE_VERSION_NUMBER = "3.0.1-camera-v4.1-field"
DEVICE_TYPE = "STELLA-Q2"
# STELLA-Q2 spectrometer instrument
# NASA open source software license
# Paul Mirel 2025
# Camera integration prototype: OV2640 JPEG + per-measurement image_file logging

quick_start = False #if False, show the startup information screens. if True, skip the startup information screens

# Legacy interval setting retained for compatibility/UI. v4 does not auto-log; sampling is button-triggered.
# set the sampling period = reciprocal of the sampling cadence
#               seconds + ( minutes ) + ( hours )
sample_interval_s = 1.0 + ( 0 * 60 ) + ( 0 * 3600 )
burst_count = 1
manual_single_shot = True  # one short button click = one spectrum + one JPEG; no automatic logging
FIELD_MODE = True  # field safety: disable USB/keyboard RTC-setting menu during normal use
logplot = True #False
screen_wait_s = 2 # how long to hold each screen during wake up

#Gain 0: 1x
#Gain 1: 3.7x
#Gain 2: 16x (sensors are factory calibrated to irradiance at this value)
#Gain 3: 64x
triad_gain_number = 2

# integration time integer set from 0 to 255
# integration time will be = 2.8ms * [integration cycles + 1]
# 59 cycles = 166ms (sensors are factory calibrated to irradiance at this value)
# max int time = 717ms
triad_intg_cycles = 59 #59

# Camera integration settings (Arducam Mini 2MP Plus / OV2640)
# The working Arducam.py on CIRCUITPY must use:
# CS=D17, SCLK=D18, MOSI=D19, MISO=D16, SDA=D20, SCL=D21.
CAMERA_ENABLED = True
CAMERA_BUFFER_SIZE = 128
CAMERA_CAPTURE_TIMEOUT_S = 3.0
CAMERA_ESTIMATED_JPEG_BYTES = 15000  # conservative estimate for SD-card time remaining display


## Manual: ##
## starts up in continuous recording
## click the button to pause recording (response is a bit slow ~1s)
## click again to take a data point burst, as many as you want.
## Set the number of measurements per burst on line 13.
## press-and-hold (2sec) to turn on the lamps,
## press-and-hold (2sec) again to turn off the lamps.
## press-and-hold (>10sec) to go into time set mode (USB to computer and serial dialogue required to set time).


import gc
gc.collect()
start_mem_free_kB = gc.mem_free()/1000
print("start memory free {0:.2f} kB".format( start_mem_free_kB ))
import time
import os
import microcontroller
import board
import digitalio
import rtc
import neopixel
import storage
import adafruit_sdcard
import busio
import math
import adafruit_max1704x
from adafruit_pcf8523 import pcf8523
import displayio
import i2cdisplaybus
import terminalio
import adafruit_displayio_ssd1306
from adafruit_display_text import label
import vectorio
from i2c_button import I2C_Button
import adafruit_gps
import AS7265X_sparkfun
from AS7265X_sparkfun import AS7265X
from Arducam_Q2 import ArducamClass, OV2640, JPEG, OV2640_320x240, ARDUCHIP_TRIG, CAP_DONE_MASK, MAX_FIFO_SIZE
print( "mem free after imports = {} kB, {} %".format(int(gc.mem_free()/1000), int(100*(gc.mem_free()/1000)/start_mem_free_kB )) )
if burst_count < 1:
    burst_count = 1

def main():
    BLUE = ( 0, 0, 255 )
    GREEN = ( 255, 0, 0 )
    YELLOW = ( 127, 255, 0 )
    RED = ( 0, 255, 0 )
    OFF = ( 0, 0, 0 )
    gc.collect()
    displayio.release_displays()
    UID = get_uid()

    # initialize hardware
    vfs = initialize_sd_card()
    if False: #sample file data
        lines = 433
        filesize_bytes = 41*1000
    if True:
        lines = 1243
        filesize_bytes = 118*1000
    bytes_per_line = filesize_bytes/ lines
    bytes_per_hour = bytes_per_line * burst_count / (sample_interval_s / 3600)  #If sensor is in continuous mode, how fast does it build up?
    if CAMERA_ENABLED:
        # Add a conservative estimate for one JPEG per spectral measurement.
        bytes_per_hour += CAMERA_ESTIMATED_JPEG_BYTES * burst_count / (sample_interval_s / 3600)
    if vfs:
        recording_time_remaining_h = evaluate_sdcard_storage(
            vfs, bytes_per_hour, verbose=True
        )
    else:
        recording_time_remaining_h = False

    i2c_bus = initialize_i2c_bus()
    onboard_neopixel = initialize_neopixel(board.NEOPIXEL)

    if vfs:
        onboard_neopixel.fill(YELLOW)
    else:
        onboard_neopixel.fill(RED)

    main_display_group = initialize_display(i2c_bus)

    # Field safety: never allow a scientific sample when the SD card is unavailable.
    if not vfs:
        fatal_sd_error(main_display_group, onboard_neopixel, RED)

    pages_list = []
    welcome_page = make_welcome_page( main_display_group )
    pages_list.append( welcome_page )
    welcome_page.show()

    # initialize devices
    battery_monitor = initialize_battery_monitor( i2c_bus )
    battery_monitor.read()
    hardware_clock = initialize_hardware_clock( i2c_bus )
    hardware_clock.sync_system_clock()
    print( "clock battery is OK:", hardware_clock.battery_ok() )
    iso8601_utc_timestamp, decimal_hour = hardware_clock.get_time_now_iso_dec()
    last_datestamp = iso8601_utc_timestamp[0:8]
    print( "timestamps:", iso8601_utc_timestamp, decimal_hour, hardware_clock.get_day_now() )
    button = initialize_button_module( i2c_bus )
    button.set_brightness( 255 )
    time.sleep(0.1)
    button.set_brightness( 1 )
    vis_nir_triad_spectrometer = initialize_vis_nir_triad_spectrometer( i2c_bus )
    vis_nir_triad_spectrometer.report()

    # Camera uses a separate hardware SPI bus (D16-D19) and bit-banged I2C (D20-D21),
    # so it does not share the Q2 SD-card SPI or Qwiic I2C bus.
    camera = False
    camera_buffer = None
    if CAMERA_ENABLED:
        camera = initialize_camera()
        if camera:
            camera_buffer = bytearray(CAMERA_BUFFER_SIZE)

    gc.collect()
    print( "memory free after device object creations = {} kB, {} %".format(int(gc.mem_free()/1000), int(100*(gc.mem_free()/1000)/start_mem_free_kB )))

    header = "iso8601_utc, UID, batch, measurement_number, decimal_hour"
    header += ", gain, integration_time_ms, wavelength_nm"
    header += ", irradiance_uw.per.cm.sq_factory_cal, irradiance.uncertainty_uw.per.cm.sq_factory_cal"
    header += ", raw_counts, detector_chip_number, detector_chip_temperature_C"
    header += ", battery_voltage, battery_percentage, image_file\n"

    # create or update the batch number
    batch_number = 0
    batch_number = update_batch( hardware_clock.read() )
    print( "batch number = {}".format( batch_number ))

    filename_in_use = update_filename( DEVICE_TYPE, hardware_clock.read(), header, batch_number )
    print( filename_in_use )
    gc.collect()


    triad_last_gain_number = triad_gain_number
    vis_nir_triad_spectrometer.set_gain_number(triad_gain_number)
    print( "triad spectrometer gain set to {} X".format( vis_nir_triad_spectrometer.check_gain_ratio() ))

    ## set triad integration time and report setting ##
    #0-255, 0-717ms # default is 57 = TBD ms.
    triad_last_intg_cycles = triad_intg_cycles
    triad_intg_time_ms = vis_nir_triad_spectrometer.set_integration_cycles(triad_intg_cycles)
    print("triad spectrometer integration time set to {} ms".format(triad_intg_time_ms))

    vis_nir_triad_spectrometer.lamps_on()
    time.sleep(1)
    vis_nir_triad_spectrometer.lamps_off()

    #ambient_air_sensor = initialize_ambient_air_sensor( i2c_bus ) #future use

    date_time_battery_page = make_date_time_battery_page( main_display_group, hardware_clock, battery_monitor )
    pages_list.append( date_time_battery_page )
    date_time_battery_page.update_values()
    if not quick_start:
        hide_all_pages( pages_list )
        date_time_battery_page.show()
        start = time.monotonic()
        while time.monotonic() < start + screen_wait_s :
            time.sleep(0.1)
            date_time_battery_page.update_values()

    interval_burst_storage_page = make_interval_burst_storage_page( main_display_group, vfs, bytes_per_hour, sample_interval_s, burst_count )
    pages_list.append( interval_burst_storage_page )
    interval_burst_storage_page.update_values()
    if not quick_start:
        hide_all_pages( pages_list )
        interval_burst_storage_page.show()
        time.sleep( screen_wait_s )

    uid_batch_page = make_uid_batch_page( main_display_group, UID, batch_number )
    pages_list.append( uid_batch_page )
    uid_batch_page.update_values( UID, batch_number )
    hide_all_pages( pages_list )
    uid_batch_page.show()
    time.sleep( screen_wait_s )

    hide_all_pages( pages_list )
    graph_group, graph_bar, graph_bar_x, polygon = create_graph_screen( main_display_group, vis_nir_triad_spectrometer.bands_sorted )
    # create log label and backing block
    log_tag_group = displayio.Group(scale=1, x=4, y=24)
    text = "log y"
    log_tag_area = label.Label(terminalio.FONT, text=text, color=0x000000)
    log_tag_group.append(log_tag_area) # Subgroup for text scaling
    main_display_group.append(log_tag_group)
    if logplot:
        log_tag_group.hidden = False
    else:
        log_tag_group.hidden = True

    number_of_pages = len( pages_list )

    operational = True
    lamps_on = False
    # Manual single-shot mode: the Q2 idles until a short button click.
    # One click creates exactly one spectral measurement and one matching JPEG.
    burst_mode = True
    acquire_burst = False
    button.clear()
    print("manual single-shot mode ready: short click = 1 spectrum + 1 JPEG")
    try:
        lowest_free_B = 1000000
        while True:
            event = button.check()
            if event:
                display_timer_start = time.monotonic()
                print(event)
                if event == "click":
                    acquire_burst = True
                elif event == "press":
                    lamps_on = not lamps_on
                    if lamps_on:
                        vis_nir_triad_spectrometer.lamps_on()
                    else:
                        vis_nir_triad_spectrometer.lamps_off()
                elif event == "long_press":
                    if FIELD_MODE:
                        print("long press ignored: field mode enabled")
                        button.clear()
                    else:
                        hide_all_pages(pages_list)
                        hardware_clock.set_time()
                        button.clear()
            gc.collect()

            if acquire_burst:
                if gc.mem_free() < lowest_free_B:
                    lowest_free_B = gc.mem_free()
                    print("new lowest memory free: {} B".format(lowest_free_B))

                # Each deliberate click gets a new batch/sample identifier.
                batch_number = update_batch(hardware_clock.read())
                uid_batch_page.update_values(UID, batch_number)
                graph_group.hidden = True
                uid_batch_page.show()

                # burst_count is fixed to 1 in manual single-shot mode.
                mm_num = 0
                spectral_data_sorted = []
                while mm_num < burst_count:
                    iso8601_utc_timestamp, decimal_hour = hardware_clock.get_time_now_iso_dec()
                    current_datestamp = iso8601_utc_timestamp[0:8]
                    if last_datestamp != current_datestamp:
                        filename_in_use = update_filename(DEVICE_TYPE, hardware_clock.read(), header, batch_number)
                        last_datestamp = current_datestamp

                    image_filename = make_camera_filename(iso8601_utc_timestamp, batch_number, mm_num)
                    image_file_for_csv = ""

                    # Start the camera exposure first. The JPEG remains in the camera FIFO
                    # while the TRIAD measurement is acquired, keeping the two observations
                    # as close together in time as practical.
                    image_fifo_length = False
                    if camera and vfs:
                        image_fifo_length = capture_camera_frame(camera)

                    vis_nir_triad_spectrometer.read()
                    battery_monitor.read()

                    # Build the graph only from this deliberately triggered spectrum.
                    spectral_data_sorted = []
                    for item in vis_nir_triad_spectrometer.bands_sorted:
                        if logplot:
                            value = vis_nir_triad_spectrometer.dict_fcal[item]
                            if value > 0:
                                spectral_data_sorted.append(math.log(value, 10))
                            else:
                                spectral_data_sorted.append(0)
                        else:
                            spectral_data_sorted.append(vis_nir_triad_spectrometer.dict_fcal[item])
                    print(spectral_data_sorted)

                    # Save the one captured JPEG to the already-mounted SD card.
                    if camera and vfs and image_fifo_length:
                        image_path = "/sd/{}".format(image_filename)
                        if save_camera_fifo_to_sd(camera, image_path, image_fifo_length, camera_buffer):
                            image_file_for_csv = image_filename

                    # Append the 18 wavelength rows for this one spectrum. Every row gets
                    # the same image filename, permanently linking the spectrum to the photo.
                    if vfs:
                        try:
                            with open("/sd/{}".format(filename_in_use), "a") as f:
                                gain_ratio = vis_nir_triad_spectrometer.check_gain_ratio()
                                if onboard_neopixel:
                                    onboard_neopixel.fill(BLUE)
                                for item in vis_nir_triad_spectrometer.bands_sorted:
                                    f.write("{}, {}, ".format(iso8601_utc_timestamp, UID))
                                    f.write("{}, {}, {}, {}, {}, ".format(batch_number, mm_num, decimal_hour, gain_ratio, int(triad_intg_time_ms)))
                                    f.write("{}, {}, {}, {}, {}, {}".format(
                                        item,
                                        vis_nir_triad_spectrometer.dict_fcal[item],
                                        vis_nir_triad_spectrometer.dict_fcal[item] * vis_nir_triad_spectrometer.uncertainty_percent/100,
                                        vis_nir_triad_spectrometer.dict_counts[item],
                                        vis_nir_triad_spectrometer.dict_chip_n[item],
                                        vis_nir_triad_spectrometer.chip_temp_c[vis_nir_triad_spectrometer.dict_chip_n[item]]
                                    ))
                                    f.write(", ")
                                    f.write(battery_monitor.log())
                                    f.write(", {}".format(image_file_for_csv))
                                    f.write("\n")
                            if onboard_neopixel:
                                onboard_neopixel.fill(OFF)
                        except OSError as err:
                            print("\nError: SD card write failed: {:}".format(err))
                            fatal_sd_error(main_display_group, onboard_neopixel, RED)

                    mm_num += 1

                # Show the spectrum that was just captured, then go idle until the next click.
                if spectral_data_sorted:
                    graph_data(spectral_data_sorted, graph_bar, graph_bar_x, polygon)
                acquire_burst = False
                uid_batch_page.hide()
                graph_group.hidden = False
                print("single-shot complete; waiting for next button click")
                gc.collect()

            # Fast idle loop for responsive button handling; no automatic spectra are acquired.
            time.sleep(0.05)

    finally:  # clean up the busses when ctrl-c'ing out of the loop
        displayio.release_displays()
        print( "displayio displays released" )
        i2c_bus.deinit()
        print( "i2c_bus deinitialized" )

def graph_data( spectral_data_sorted, graph_bar, graph_bar_x, polygon ):
    if spectral_data_sorted:
        y_zero_pixels = 32
        y_upper_pixels = 0
        graph_points = [(2, 112), (2, 104)]
        y_span_pixels = y_zero_pixels - y_upper_pixels
        irradiances = spectral_data_sorted
        irrad_min = min( irradiances )
        irrad_max = max( irradiances )
        irrad_span = irrad_max - irrad_min
        if irrad_span < 1:
            irrad_span = 1
        for count in range (len( irradiances )):
            irrad_value = irradiances[ count ]
            irrad_height = irrad_value - irrad_min
            irrad_normalized_height = irrad_height/ irrad_span
            irrad_bar_height_pixels = int(y_span_pixels * irrad_normalized_height)
            irrad_drop_height_pixels = int(y_span_pixels - irrad_bar_height_pixels)
            irrad_y_top_pixel = int(y_upper_pixels + irrad_drop_height_pixels)
            graph_bar[count].y = irrad_y_top_pixel
            graph_bar[count].height = 1
            point = (graph_bar_x[count]+1, irrad_y_top_pixel)
            graph_points.append(point)
        graph_points.append((126, 104))
        graph_points.append((126, 112))
        polygon.points = graph_points
        #del y_zero_pixels, y_upper_pixels, graph_points, y_span_pixels, irradiances, spectral_data_sorted
        #del irrad_min, irrad_max, irrad_span, count, irrad_value, irrad_height
        #del irrad_normalized_height, irrad_bar_height_pixels, irrad_drop_height_pixels
        #del irrad_y_top_pixel, point

def create_graph_screen( main_display_group, bands_sorted ):
    if main_display_group is not False:
        graph_group = displayio.Group()
        palette = displayio.Palette(1)
        palette[0] = 0xFFFFFF
        points2 = [ (2, 32), (2, 30), (126, 30), (126, 32)]
        polygon = vectorio.Polygon(pixel_shader=palette, points=points2, x=0, y=0)
        graph_group.append( polygon )
        bar_color = 0xFFFFFF
        bar = displayio.Palette(1)
        bar[0] = bar_color
        bar_width = 1
        bar_default_height = 1
        graph_bar_x = (0, 6, 12, 18, 24, 29, 35, 41, 47, 55, 63, 69, 75, 82, 93, 105, 114, 124)
        graph_bar=[]
        for count in range( len( bands_sorted)):
            graph_bar.append( vectorio.Rectangle(pixel_shader=bar, width=bar_width, height=bar_default_height, x=graph_bar_x[count], y=106))
            graph_group.append( graph_bar[count] )
        print( "initialized graph screen" )
        main_display_group.append( graph_group )
        return ( graph_group, graph_bar, graph_bar_x, polygon )
    else:
        print( "graph screen initialization failed" )
        return False, False, False, False, False, False, False, False, False, False, False, False


class Page:
    def __init__( self ):
        pass
    def show(self):
        self.group.hidden = False
    def hide(self):
        self.group.hidden = True
    def update_values(self):
        pass

class Welcome_Page( Page ):
    def __init__( self ):
        super().__init__()
    def make_group( self ):
        self.group = displayio.Group()
        text = "{}".format(DEVICE_TYPE)
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=2, x=8, y=8 )
        text_group.append( text_area )
        self.group.append( text_group )

        text = "software v{}".format(SOFTWARE_VERSION_NUMBER)
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=14, y=25 )
        text_group.append( text_area )
        self.group.append( text_group )
        return self.group
    def update_values( self ):
        pass

def make_welcome_page( main_display_group ):
    welcome_page = Welcome_Page()
    group = welcome_page.make_group()
    main_display_group.append( group )
    welcome_page.hide()
    return welcome_page

class Date_Time_Battery_Page( Page ):
    def __init__( self, hardware_clock, battery_monitor):
        super().__init__()
        self.hardware_clock = hardware_clock
        self.battery_monitor = battery_monitor
    def make_group( self ):
        self.group = displayio.Group()
        date_text = "YYYY-MM-DD"
        self.date_text_area = label.Label( terminalio.FONT, text=date_text, color=0xFFFFFF )
        date_text_group = displayio.Group( scale=1, x=4, y=8 ) #( scale=2, x=4, y=8 )
        date_text_group.append( self.date_text_area )
        self.group.append( date_text_group )
        time_text = "HH:MM:SS Z"
        self.time_text_area = label.Label( terminalio.FONT, text=time_text, color=0xFFFFFF )
        time_text_group = displayio.Group( scale=1, x=4, y=22 )
        time_text_group.append( self.time_text_area )
        self.group.append( time_text_group )
        text = "battery"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=78, y=8)
        text_group.append( text_area )
        self.group.append( text_group )
        battery_soc_text = "000 %"
        self.battery_soc_text_area = label.Label( terminalio.FONT, text=battery_soc_text, color=0xFFFFFF )
        battery_soc_text_group = displayio.Group( scale=1, x=86, y=22 )
        battery_soc_text_group.append( self.battery_soc_text_area )
        self.group.append( battery_soc_text_group )
        return self.group

    def update_values( self ):
        self.hardware_clock.read()
        self.battery_monitor.read()
        self.date_text_area.text = "{}-{:02}-{:02}".format( self.hardware_clock.timenow.tm_year, self.hardware_clock.timenow.tm_mon, self.hardware_clock.timenow.tm_mday )
        self.time_text_area.text = "{:02}:{:02}:{:02} Z".format( self.hardware_clock.timenow.tm_hour, self.hardware_clock.timenow.tm_min, self.hardware_clock.timenow.tm_sec )
        self.battery_soc_text_area.text = "{} %".format( int(self.battery_monitor.percentage ))

def make_date_time_battery_page( main_display_group, hardware_clock, battery_monitor ):
    date_time_battery_page = Date_Time_Battery_Page(  hardware_clock, battery_monitor )
    group = date_time_battery_page.make_group()
    main_display_group.append( group )
    date_time_battery_page.hide()
    return date_time_battery_page

class Interval_Burst_Storage_Page( Page ):
    def __init__( self, vfs, bytes_per_hour, interval, burst ):
        super().__init__()
        self.vfs = vfs
        self.bytes_per_hour = bytes_per_hour
        self.interval = interval
        self.burst = burst
    def make_group( self ):
        self.group = displayio.Group()
        text = "Intrvl:"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=0, y=8 )
        text_group.append( text_area )
        self.group.append( text_group )
        interval_text = "00"
        self.interval_text_area = label.Label( terminalio.FONT, text= interval_text, color=0xFFFFFF )
        interval_text_group = displayio.Group( scale=1, x=44, y=8 )
        interval_text_group.append( self.interval_text_area )
        self.group.append( interval_text_group )
        text = "Burst:"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=0, y=22 )
        text_group.append( text_area )
        self.group.append( text_group )
        burst_text = "0"
        self.burst_text_area = label.Label( terminalio.FONT, text= burst_text, color=0xFFFFFF )
        burst_text_group = displayio.Group( scale=1, x=44, y=22 )
        burst_text_group.append( self.burst_text_area )
        self.group.append( burst_text_group )
        text = "SD free:"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=78, y=8)
        text_group.append( text_area )
        self.group.append( text_group )
        remaining_time_text = ""
        self.remaining_time_text_area = label.Label( terminalio.FONT, text=remaining_time_text, color=0xFFFFFF )
        remaining_time_text_group = displayio.Group( scale=1, x=74, y=22 )
        remaining_time_text_group.append( self.remaining_time_text_area )
        self.group.append( remaining_time_text_group )
        return self.group

    def update_values( self ):
        if self.interval < 10:
            self.interval_text_area.text = "{}s".format(self.interval)
        else:
            self.interval_text_area.text = "{}s".format(int(self.interval))
        self.burst_text_area.text = "{}".format(self.burst)
        recording_time_remaining_h = evaluate_sdcard_storage( self.vfs, self.bytes_per_hour, verbose = False )
        if not self.vfs:
            self.remaining_time_text_area.text = "No SDcard"
        else:
            if recording_time_remaining_h < 24:
                self.remaining_time_text_area.text = "{} hours".format( recording_time_remaining_h )
            elif recording_time_remaining_h < 240:
                self.remaining_time_text_area.text = "{} days".format( round(recording_time_remaining_h/24,1) )
            elif recording_time_remaining_h < 240 * 365.25:
                self.remaining_time_text_area.text = "{} days".format( int(recording_time_remaining_h/24) )
            else:
                self.remaining_time_text_area.text = "{} years".format( int(recording_time_remaining_h/24/365.25) )

def make_interval_burst_storage_page( main_display_group, vfs, bytes_per_hour, interval, burst):
    interval_burst_storage_page = Interval_Burst_Storage_Page( vfs, bytes_per_hour, interval, burst )
    group = interval_burst_storage_page.make_group()
    main_display_group.append( group )
    interval_burst_storage_page.hide()
    return interval_burst_storage_page

class Uid_Batch_Page( Page ):
    def __init__( self, UID, batch_number):
        super().__init__()
        self.UID = UID
        self.batch_number = batch_number
    def make_group( self ):
        self.group = displayio.Group()
        text = "UID"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=4, y=4)
        text_group.append( text_area )
        self.group.append( text_group )
        text = "Batch"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=1, x=78, y=4)
        text_group.append( text_area )
        self.group.append( text_group )
        batch_number_text = "00"
        self.batch_number_text_area = label.Label( terminalio.FONT, text=batch_number_text, color=0xFFFFFF )
        batch_number_text_group = displayio.Group( scale=2, x=78, y=24 ) #( scale=2, x=4, y=8 )
        batch_number_text_group.append( self.batch_number_text_area )
        self.group.append( batch_number_text_group )
        uid_text = "0000"
        self.uid_text_area = label.Label( terminalio.FONT, text=uid_text, color=0xFFFFFF )
        uid_text_group = displayio.Group( scale=2, x=4, y=24 ) #( scale=2, x=4, y=8 )
        uid_text_group.append( self.uid_text_area )
        self.group.append( uid_text_group )
        return self.group

    def update_values( self, UID, batch_number ):
        self.uid_text_area.text = "{}".format(UID)
        if int(batch_number) < 10:
            self.batch_number_text_area.text = "{}".format(batch_number)
        elif int(batch_number) < 100:
            self.batch_number_text_area.text = "{}".format(batch_number)
        else:
            self.batch_number_text_area.text = "{}".format(batch_number)

def make_uid_batch_page( main_display_group, UID, batch_number ):
    uid_batch_page = Uid_Batch_Page(  UID, batch_number )
    group = uid_batch_page.make_group()
    main_display_group.append( group )
    uid_batch_page.hide()
    return uid_batch_page

''' # future sensor status screen
class Ambient_Page( Page ):
    def __init__( self, ambient_air_sensor ):
        super().__init__()
        self.ambient_air_sensor = ambient_air_sensor
    def make_group( self ):
        temperature_text_y = 68
        text = "T"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=3, x=0, y=temperature_text_y )
        text_group.append( text_area )
        self.group.append( text_group )
        text = "--.-"
        self.temperature_text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        temperature_text_group = displayio.Group( scale=3, x=30, y=temperature_text_y )
        temperature_text_group.append( self.temperature_text_area )
        self.group.append( temperature_text_group )
        text = "C"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=2, x=110, y=temperature_text_y+3 )
        text_group.append( text_area )
        self.group.append( text_group )
        humidity_text_y = temperature_text_y + 46
        text = "RH"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=3, x=0, y=humidity_text_y )
        text_group.append( text_area )
        self.group.append( text_group )
        text = "--"
        self.humidity_text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        humidity_text_group = displayio.Group( scale=3, x=48, y=humidity_text_y )
        humidity_text_group.append( self.humidity_text_area )
        self.group.append( humidity_text_group )
        text = "%"
        text_area = label.Label( terminalio.FONT, text=text, color=0xFFFFFF )
        text_group = displayio.Group( scale=2, x=110, y=humidity_text_y+3 )
        text_group.append( text_area )
        self.group.append( text_group )
        return self.group
    def update_values( self ):
        if self.ambient_air_sensor.temperature_C is not None:
            self.temperature_text_area.text = "{}".format(round(self.ambient_air_sensor.temperature_C,1))
            humid_value = int( self.ambient_air_sensor.humidity_percent )
            if humid_value < 100:
                self.humidity_text_area.text = " {}".format( humid_value )
            else:
                self.humidity_text_area.text = "{}".format( humid_value )

def make_ambient_page( main_display_group, ambient_air_sensor ):
    ambient_page = Ambient_Page( ambient_air_sensor )
    group = ambient_page.make_group()
    main_display_group.append( group )
    ambient_page.hide()
    return ambient_page
'''








def hide_all_pages( pages_list ):
    for page in pages_list:
        page.hide()

def update_filename(device_type, timenow, new_header, batch_number):
    if timenow is not None:
        create_new_file = False
        filename_of_the_day = ("{}_data_{}{:02}{:02}-{}.csv".format(device_type, timenow.tm_year,timenow.tm_mon,timenow.tm_mday,batch_number))
        # create a dummy filename
        last_filename_in_use = ("{}_data_{}{:02}{:02}-{}.csv".format(device_type, 2000,1,1,0))
        # look up today's date
        current_datestamp = "{:04}{:02}{:02}".format( timenow.tm_year, timenow.tm_mon, timenow.tm_mday)
        previous_header = False
        # look up the last filename
        try:
            with open( "/sd/last_filename.txt", "r" ) as lfn:
                try:
                    last_filename_in_use = lfn.readline().rstrip()
                except ValueError as err:
                    print(err)
        except OSError:
            print( "last_filename.txt file not found, creating new last_filename.txt file" )
            try:
                with open ( "/sd/last_filename.txt", "w" ) as lfn:
                    lfn.write(filename_of_the_day)
            except:
                print( "unable to create last_filename.txt file")

        substrings = last_filename_in_use.split("_")
        date_batch = substrings[-1]
        substrings = date_batch.split("-")
        last_datestamp = substrings[0]
        if last_datestamp != current_datestamp:
            print( "new day, start a new file" )
            create_new_file = True
        else:
            try:
                with open( "/sd/{}".format(last_filename_in_use), "r" ) as lfn:
                    previous_header = lfn.readline().rstrip()
                    previous_header += "\n"
            except OSError as err:
                print( err )
            if new_header != previous_header:
                print( "configuration change, start a new file" )
                create_new_file = True
            else:
                filename_to_use = last_filename_in_use
        if create_new_file:
            filename_to_use = filename_of_the_day
            try:
                with open( "/sd/{}".format(filename_to_use), "w" ) as fn:
                    fn.write( new_header )
            except OSError as err:
                print( err )
            try:
                with open ( "/sd/last_filename.txt", "w" ) as lfn:
                    lfn.write(filename_to_use)
            except:
                print( "unable to write to last_filename.txt file")

    else:
        filename_to_use = "{}_data_no_timestamp.csv".format(DEVICE_TYPE)
        try:
            with open( "/sd/{}".format(filename_to_use), "w" ) as fn:
                fn.write( new_header )
        except OSError as err:
            print( err )

    return filename_to_use

def update_batch( timestamp ):
    if timestamp is not None:
        datestamp = "{:04}{:02}{:02}".format( timestamp.tm_year, timestamp.tm_mon, timestamp.tm_mday)
        try:
            with open( "/sd/batch.txt", "r" ) as b:
                try:
                    previous_batchfile_string = b.readline()
                    previous_datestamp = previous_batchfile_string[ 0:8 ]
                    previous_batch_number = int( previous_batchfile_string[ 8: ])
                except ValueError:
                    previous_batch_number = 0
                if datestamp == previous_datestamp:
                    # this is the same day, so increment the batch number
                    batch_number = previous_batch_number + 1
                else:
                    # this is a different day, so start the batch number at 0
                    batch_number = 0
        except OSError:
                print( "batch.txt file not found" )
                batch_number = 0

        batch_string = ( "{:03}".format( batch_number ))
        batch_file_string = datestamp + batch_string
        try:
            with open( "/sd/batch.txt", "w" ) as b:
                b.write( batch_file_string )
                b.write("\n")
        except OSError as err:
            print("Error: writing batch.txt {:}".format(err) )
            pass
        batch_string = ( "{:}".format( batch_number ))
    else:
        batch_string = ( "{:}".format( 0 ))
    return batch_string

def timestamp_to_decimal_hour( timestamp ):
    try:
        decimal_hour = timestamp.tm_hour + timestamp.tm_min/60.0 + timestamp.tm_sec/3600.0
        return decimal_hour
    except ValueError as err:
        print( "Error: invalid timestamp: {:}".format(err) )
        return False

class Device: #parent class
    def __init__(self, name = None, pn = None, address = None, swob = None ):
        self.name = name
        self.swob = swob
        self.pn = pn
        self.address = address
    def report(self):
        if self.swob is not None:
            print("report:", hex(self.address), self.pn, "\t", self.name, "found" )
    def found(self):
        if self.swob is not None:
            return True
        else:
            return False

def initialize_vis_nir_triad_spectrometer( i2c_bus ):
    triad_spectrometer = Null_Triad_Spectrometer()
    try:
        triad_spectrometer = as7265x_Triad_Spectrometer( i2c_bus )
        #print( "triad spectrometer initialized" )
    except:
        #print( "triad spectrometer failed to initialize" )
        pass
    return triad_spectrometer

class as7265x_Triad_Spectrometer( Device ):
    def __init__( self, com_bus ):
        super().__init__(name = "vis_nir_triad_spectrometer", pn = "as7256x", address = 0x49, swob = AS7265X( com_bus ))
        if self.swob:
            self.swob.disable_indicator()
            self.swob.set_measurement_mode(AS7265X_sparkfun.MEASUREMENT_MODE_6CHAN_CONTINUOUS)
            self.bands = 610, 680, 730, 760, 810, 860, 560, 585, 645, 705, 900, 940, 410, 435, 460, 485, 510, 535
            self.chip_n = 1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3
            self.dict_chip_n = {key:value for key, value in zip(self.bands, self.chip_n )}
            self.bands_sorted = sorted( self.bands )
            self.uncertainty_percent = 12
            self.bw_fwhm_nm = 20
            self.afov_deg = (20.5 * 2) #datasheet reports half angle.
    def check_gain_ratio(self):
        gain_number = self.swob._gain
        if gain_number < 1:
            gain_ratio = 1
        elif gain_number == 1:
            gain_ratio = 3.7
        elif gain_number == 2:
            gain_ratio = 16
        elif gain_number == 3:
            gain_ratio = 64
        return gain_ratio
    def set_gain_number(self, gain_number):
        if gain_number in range (0,4):
            self.swob.set_gain( gain_number )
        else:
            print( "out of range: set gain number to 0-3 to get gain_ratios of 1, 3.7, 16, 64" )
    def set_integration_cycles( self, cycles ):
        if cycles in range (0, 256):
            self.swob.set_integration_cycles(cycles)
            intg_time_ms = int(round((2.8*(cycles+1)),0))
        else:
            print( "out of range: set integration cycles to 0-255 for 0-717ms integration time." )
            intg_time_ms = None
        return intg_time_ms
    def found(self):
        print("found", self.pn, self.swob)
    def read(self):
        self.chip_temp_c = {1:self.swob.get_temperature(1), 2:self.swob.get_temperature(2), 3:self.swob.get_temperature(3)}
        self.data_counts = self.swob.get_value(0) # 0th position raw counts, bands unsorted order
        # VV create a dictionary where key = WL and value = raw counts
        self.dict_counts = {key:value for key, value in zip(self.bands, self.data_counts)}
        self.data_fcal = self.swob.get_value(1) # 1th position factory calibrated irrad value, bands unsorted order
        # VV create a dictionary where key = WL and value = factory cal irradiance
        self.dict_fcal = {key:value for key, value in zip(self.bands, self.data_fcal)}
        #self.dict_uncty_fcal = {key:value for key, value in zip(self.bands, (self.data_fcal*self.uncert_percent/100))}
        # >> self.dict_scal = {key:value for key, value in zip(self.bands, 0)}
        #self.dict_uncty_scal = {key:value for key, value in zip(self.bands, (0))}
    def list_channels():
        return self.bands_sorted
    def header( self ):
        return "WL.nm, irrad.uW/(cm^2), irrad.uncty.uW/(cm^2), counts, chip_num, chip_temp_C"
    def log( self ):
        print( self.data_counts )
        print( self.data_fcal )
        #self.irradiance[ch] = self.data[ch]/self.tsis_cal_counts_per_irradiance[ch]
        #self.irradiance[ch] = self.data[ch]/self.steno_cal_counts_per_irradiance[ch]
        #return "{}, {}, {}, {}".format( self.center_wavelengths_nm[ch], self.data[ch],
        #    self.irradiance[ch], self.irradiance[ch]*self.calibration_error )
    def printlog(self,ch):
        print( self.log(ch) )
    def lamps_on(self):
        #print( "turn on the lamps")
        self.swob.enable_bulb(0)   # white
        self.swob.enable_bulb(1)   # NIR
        self.swob.enable_bulb(2)   # UV
    def lamps_off(self):
        #print( "turn off the lamps")
        self.swob.disable_bulb(0)   # white
        self.swob.disable_bulb(1)   # NIR
        self.swob.disable_bulb(2)   # UV

class Null_Triad_Spectrometer():
    def __init__( self ):
        self.swob = None
        self.bands = [0,0]
        self.bands_sorted = [0,0]   # empty list
        self.dict_chip_n = [0,0]
        self.chip_temps = [0,0]
        self.dict_fcal = {0:0}      # empty dictionary
        self.dict_counts = {0:0}
        self.uncertainty_percent = 0
        self.chip_temp_c = {1:0, 2:0, 3:0}
        self.chip_n = 1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3
        self.dict_chip_n = {key:value for key, value in zip(self.bands, self.chip_n )}

    def check_gain_ratio(self):
        return 1
    def set_gain_number(self, gain_number):
        pass
    def found(self):
        pass
    def read(self):
        pass
    def log(self):
        pass
    def report(self):
        pass
    def printlog(self):
        pass
    def blink(self, duration):
        pass
    def header(self):
        pass
    def lamps_on(self):
        pass
    def lamps_off(self):
        pass
    def set_integration_cycles(self, cycles):
        return 1

''' #future sensor capability
def initialize_ambient_air_sensor( i2c_bus ):
    ambient_air_sensor = Null_Air_Sensor()
    try:
        ambient_air_sensor = bme280_Air_Sensor( i2c_bus )
    except NameError as err:
        print( "library missing:", err )
    except Exception:
        pass
    return ambient_air_sensor

class bme280_Air_Sensor( Device ):
    #https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    def __init__( self, com_bus ):
        super().__init__(name = "ambient_air_sensor", pn = "bme280", address = 0x77, swob = adafruit_bme280.Adafruit_BME280_I2C( com_bus ))
        self.temperature_C = None
        self.temperature_uncertainty_C = 0.5
        self.pressure_hPa = None
        self.pressure_uncertainty_hPa = 1
        self.altitude_m = None
        self.altitude_uncertainty_m = 1000
        self.humidity_percent = None
        self.humidity_uncertainty_percent = 3
    def read(self):
        self.temperature_C = self.swob.temperature
        self.pressure_hPa = self.swob.pressure
        self.humidity_percent = self.swob.relative_humidity
        self.altitude_m = self.swob.altitude
        # TD: =243.04*(LN(RH/100)+((17.625*T)/(243.04+T)))/(17.625-LN(RH/100)-((17.625*T)/(243.04+T)))
        # from https://bmcnoldy.earth.miami.edu/Humidity.html
        #self.dewpoint_C = 0 #self.temperature -((100-self.humidity)/5) #update this to the formula above.
        #self.dewpoint_uncertainty_C = 3.2
    def log(self):
        return "{}, {}, {}, {}, {}, {}, {}, {}".format(
            round(self.temperature_C, 3), self.temperature_uncertainty_C,
            round(self.humidity_percent, 1), self.humidity_uncertainty_percent,
            #round(self.dewpoint_C, 1), self.dewpoint_uncertainty_C,
            round(self.pressure_hPa, 0), self.pressure_uncertainty_hPa,
            round(self.altitude_m, 3), self.altitude_uncertainty_m )
    def printlog(self):
        print( self.log())
    def header(self):
        #return( "air_temp_C, air_temp_uncertainty_C, air_humid_%, air_humid_uncertainty_%, air_dewpoint_C, air_dp_uncertainty_C, air_press_hPa, air_press_uncertainty_hPa, press_alt_m, press_alt_uncertainty_m" )
        return( "air_temp_C, air_temp_uncertainty_C, air_humid_%, air_humid_uncertainty_%, air_press_hPa, air_press_uncertainty_hPa, press_alt_m, press_alt_uncertainty_m" )

class Null_Air_Sensor(Device):
    def __init__( self ):
        self.swob = None
        self.temperature_C = None
        self.temperature_uncertainty_C = None
        self.pressure_hPa = None
        self.pressure_uncertainty_hPa = None
        self.altitude_m = None
        self.altitude_uncertainty_m = None
        self.humidity_percent = None
        self.humidity_uncertainty_percent = None
    def read(self):
        pass
    def log(self):
        pass
    def report(self):
        pass
    def printlog(self):
        pass
    def header(self):
        pass
'''

def initialize_button_module( i2c_bus ):
    button_module = Null_Button_Module()
    try:
        button_module = qwiic_button_Button_Module( i2c_bus )
    except NameError as err:
        print( "library missing:", err )
    except Exception:
        pass
    return button_module

class qwiic_button_Button_Module( Device ):
    def __init__( self, com_bus ):
        super().__init__(name = "button_module", pn = "qwiic_button", address = 0x6F, swob = I2C_Button( com_bus ))
        self.swob.led_bright = 1
    def read(self):
        pass
    def header(self):
        pass
    def set_brightness( self, brightness_setting ):
        if brightness_setting > 255:
            brightness_setting = 255
        if brightness_setting < 0:
            brightness_setting = 0
        self.swob.led_bright = brightness_setting
    def clear( self ):
        self.swob.clear()
    def check( self ):
        click_limit_s = 1
        press_limit_s = 4
        long_press_limit_s = 6
        timeout_duration_s = 7
        observation_interval_s = 0.1
        event = False
        button_event = False
        hold_time = 0
        start_time = time.monotonic()
        button_values = self.swob.status
        if button_values[1] or button_values[ 2 ]:
            self.set_brightness( 64 )
        while button_values[1] or button_values[ 2 ] and hold_time < timeout_duration_s : # button pressed one way or another
            event = True
            self.swob.clear()
            time.sleep(observation_interval_s)
            button_values = self.swob.status
            time.sleep(observation_interval_s)
            hold_time = time.monotonic() - start_time
        if event:
            #print( hold_time )
            if hold_time < click_limit_s:
                button_event = "click"
            elif hold_time < press_limit_s:
                button_event = "press"
            elif hold_time > long_press_limit_s :
                button_event = "long_press"
            event = False
        self.swob.clear()
        self.set_brightness( 1 )
        return button_event

class Null_Button_Module():
    def __init__( self ):
        self.swob = None
    def read(self):
        pass
    def set_brightness( self, brightness_setting ):
        pass

def initialize_hardware_clock( i2c_bus ):
    hardware_clock = Null_Hardware_Clock()
    try:
        hardware_clock = pcf8523_Hardware_Clock( i2c_bus )
    except NameError as err:
        print( "library missing:", err )
    except Exception:
        pass
    return hardware_clock

class pcf8523_Hardware_Clock( Device ):
    def __init__( self, com_bus ):
        super().__init__(name = "hardware_clock", pn = "pcf8523", address = 0x68, swob = pcf8523.PCF8523( com_bus ))
        self.null_time = time.struct_time(( 2020,  01,   01,   00,  00,  00,   0,   -1,    -1 ))
        self.timenow = self.null_time
        self.DAYS = { 0:"Sunday", 1:"Monday", 2:"Tuesday", 3:"Wednesday", 4:"Thursday", 5:"Friday", 6:"Saturday" }
    def battery_ok( self ):
        try:
            self.clock_battery_ok = not self.swob.battery_low
        except:
            self.clock_battery_ok = False
        return self.clock_battery_ok
    def read(self):
        try:
            self.timenow = self.swob.datetime
        except:
            self.timenow = self.null_time
        if self.timenow.tm_wday not in range ( 0, 7 ):
            self.datetime = null_time
        return self.timenow
    def get_time_now_iso_dec( self ):
        self.read()
        iso8601_utc_timestamp = "{:04}{:02}{:02}T{:02}{:02}{:02}Z".format(
            self.timenow.tm_year, self.timenow.tm_mon, self.timenow.tm_mday,
            self.timenow.tm_hour, self.timenow.tm_min, self.timenow.tm_sec )
        decimal_hour = self.timenow.tm_hour + self.timenow.tm_min/60.0 + self.timenow.tm_sec/3600.0
        return iso8601_utc_timestamp, decimal_hour
    def get_day_now( self ):
        return self.DAYS[self.timenow.tm_wday]
    def sync_system_clock(self):
        self.read()
        try:
            system_clock = rtc.RTC()
            system_clock.datetime = self.swob.datetime
            print( "system clock synchronized to hardware clock" )
        except:
            print( "failed to synchronize system clock to hardware clock" )
    def header(self):
        return "TBD"
    def log(self):
        return " "
    def printlog(self):
        print( self.log())
    def set_time(self):
        timenow = self.swob.datetime
        print()
        weekday = timenow.tm_wday
        year = timenow.tm_year
        month = timenow.tm_mon
        day = timenow.tm_mday
        hour = timenow.tm_hour
        minute = timenow.tm_min
        second = timenow.tm_sec
        previous_weekday = weekday
        previous_year = year
        previous_month = month
        previous_day = day
        previous_hour = hour
        previous_minute = minute
        previous_second = second
        try:
            print( "The date is %s %d-%d-%d" % ( self.DAYS[ weekday ], year, month, day ))
            print( "The time is %d:%02d:%02d" % ( hour, minute, second ))
        except IndexError as err:
            print( "The clock has not been set, and the values are out of range." )

        print()
        print( "Current year is {}. Enter a new year and press return, or press return to skip.".format(timenow.tm_year))
        print(">", end = ' ')
        input_string = False
        while input_string == False:
            input_string = input().strip()
        try:
            input_integer = int( input_string )
        except ValueError:
            input_integer = 2000
        if input_integer in range (2010, 2100):
            year = input_integer
            t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
            self.swob.datetime = t

        print()
        print( "Current month is {}. Enter a new month and press return, or press return to skip.".format(timenow.tm_mon))
        print(">", end = ' ')
        input_string = False
        while input_string == False:
            input_string = input().strip()
        try:
            input_integer = int( input_string )
        except ValueError:
            pass
        if input_integer in range (1, 13):
            month = input_integer
            t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
            self.swob.datetime = t
        print()
        print( "Current day is {}. Enter a new day and press return, or press return to skip.".format(timenow.tm_mday))
        print(">", end = ' ')
        input_string = False
        while input_string == False:
            input_string = input().strip()
        try:
            input_integer = int( input_string )
        except ValueError:
            pass
        if input_integer in range (1, 32):
            day = input_integer
            t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
            self.swob.datetime = t

        print()
        print( "Current hour is {} UTC. Enter a new hour and press return, or press return to skip.".format(timenow.tm_hour))
        print(">", end = ' ')
        input_string = False
        while input_string == False:
            input_string = input().strip()
        try:
            input_integer = int( input_string )
        except ValueError:
            pass
        if input_integer in range (0, 24):
            hour = input_integer
            t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
            self.swob.datetime = t

        print()
        print( "Current minute is {}. Enter a new minute and press return, or press return to skip.".format(timenow.tm_min))
        print(">", end = ' ')
        input_string = False
        while input_string == False:
            input_string = input().strip()
        try:
            input_integer = int( input_string )
        except ValueError:
            pass
        if input_integer in range (0, 60):
            minute = input_integer
            t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
            self.swob.datetime = t

        print()
        print( "Current second is {}. Enter a new second and press return, or press return to skip.".format(timenow.tm_sec))
        print(">", end = ' ')
        input_string = False
        while input_string == False:
            input_string = input().strip()
        try:
            input_integer = int( input_string )
        except ValueError:
            pass
        if input_integer in range (0, 60):
            second = input_integer
            t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
            self.swob.datetime = t

        print()
        print( "Current weekday is {}. Enter a new weekday and press return, or press return to skip.".format(self.DAYS[timenow.tm_wday]))
        print( "Enter: sun, mon, tue, wed, thu, fri, sat" )
        print(">", end = ' ')
        input_string = False

        input_string = input().strip()
        print(input_string)
        if input_string == "mon":
            weekday = 1
        elif input_string == "tue":
            weekday = 2
        elif input_string == "wed":
            weekday = 3
        elif input_string == "thu":
            weekday = 4
        elif input_string == "fri":
            weekday = 5
        elif input_string == "sat":
            weekday = 6
        elif input_string == "sun":
            weekday = 0
        else:
            pass
        print("weekday = {}".format(weekday))
        t = time.struct_time(( year,  month,   day,   hour,  minute,  second,   weekday,   -1,    -1 ))
        self.swob.datetime = t
        print( "returning to status page" )

class Null_Hardware_Clock():
    def __init__( self ):
        self.swob = None
        self.null_time = time.struct_time(( 2020,  01,   01,   00,  00,  00,   0,   -1,    -1 ))
        self.timenow = self.null_time
    def read(self):
        pass
    def battery_ok( self ):
        pass
    def get_day_now( self ):
        pass
    def get_time_now_iso_dec( self ):
        self.read()
        iso8601_utc_timestamp = "{:04}{:02}{:02}T{:02}{:02}{:02}Z".format(
            self.timenow.tm_year, self.timenow.tm_mon, self.timenow.tm_mday,
            self.timenow.tm_hour, self.timenow.tm_min, self.timenow.tm_sec )
        decimal_hour = time.monotonic()/ 3600
        return iso8601_utc_timestamp, decimal_hour
    def sync_system_clock(self):
        pass
    def set_time(self):
        pass

def initialize_battery_monitor( i2c_bus ):
    battery_monitor = Null_Battery_Monitor()
    try:
        battery_monitor = max1704x_Battery_Monitor( i2c_bus )
    except:
        pass
    return battery_monitor

class max1704x_Battery_Monitor( Device ): #child class ( parent class ):
    def __init__( self, com_bus ):
        super().__init__(name = "battery_monitor", pn = "max1704x", address = 0x36, swob = adafruit_max1704x.MAX17048( com_bus ))
    def found(self):
        print("found", self.pn, self.swob)
    def read(self):
        self.voltage = self.swob.cell_voltage
        self.percentage = round(self.swob.cell_percent, 1)
    def header(self):
        return "bat.V, bat.%"
    def log(self):
        return "{}, {}".format( self.voltage, self.percentage )
    def printlog(self):
        print( self.log())

class Null_Battery_Monitor():
    def __init__( self ):
        self.swob = None
    def found(self):
        pass
    def read(self):
        self.voltage = 0
        self.percentage = 0
    def log(self):
        return "{}, {}".format( self.voltage, self.percentage )
        pass
    def report(self):
        pass
    def printlog(self):
        pass
    def header(self):
        pass


''' #Future display
def initialize_display( i2c_bus ):
    try:
        display_bus = I2CDisplayBus(i2c_bus, device_address=0x3D)
        display = SH1107(display_bus, width=128, height=128, display_offset=DISPLAY_OFFSET_ADAFRUIT_128x128_OLED_5297, rotation=0)
        print( "display initialized" )
    except:
        print( "display failed to initialize" )
        display = False
        display_group = False
    if display:
        display_group = displayio.Group()
        display.root_group = display_group
    return display_group
'''

def initialize_display( i2c_bus ):
    try:
        display_bus = i2cdisplaybus.I2CDisplayBus( i2c_bus, device_address=0x3c )
        display = adafruit_displayio_ssd1306.SSD1306( display_bus, width=128, height=32 )
        display_group = displayio.Group()
        print( "128x32 display initialized" )
    except:
        print( "display failed to initialize" )
        display = False
        display_group = False
    if display:
        display_group = displayio.Group()
        display.root_group = display_group
    return display_group

def fatal_sd_error(main_display_group, onboard_neopixel, red_color):
    """Show a permanent SD-card lockout on the OLED and stop sampling."""
    print("FIELD LOCKOUT: SD card unavailable")
    print("Reinsert SD card and reset the Q2.")

    if onboard_neopixel:
        onboard_neopixel.fill(red_color)

    if main_display_group is not False:
        # Clear anything already being shown on the OLED.
        while len(main_display_group) > 0:
            main_display_group.pop()

        error_group = displayio.Group()

        title = label.Label(
            terminalio.FONT,
            text="SD CARD ERROR",
            color=0xFFFFFF
        )
        title_group = displayio.Group(scale=1, x=25, y=9)
        title_group.append(title)
        error_group.append(title_group)

        help_text = label.Label(
            terminalio.FONT,
            text="REINSERT + RESET",
            color=0xFFFFFF
        )
        help_group = displayio.Group(scale=1, x=16, y=24)
        help_group.append(help_text)
        error_group.append(help_group)

        main_display_group.append(error_group)

    # Stop all operation. A power cycle/reset is required after the SD card is fixed.
    while True:
        time.sleep(1)


def initialize_neopixel( pin ):
    try:
        num_pixels = 1
        ORDER = neopixel.RGB
        neopixel_instance = neopixel.NeoPixel( pin, num_pixels, brightness=0.3, auto_write=True, pixel_order=ORDER )
        print( "neopixel initialized" )
    except:
        neopixel_instance = False
        print( "neopixel failed to initialize" )
    return neopixel_instance

def initialize_i2c_bus():
    try:
        i2c_bus = board.I2C()
        print( "i2c bus initialized" )
    except:
        print( "i2c bus failed to initialize" )
        i2c_bus = False
    return i2c_bus

def evaluate_sdcard_storage( vfs, bytes_per_hour, verbose ):
    try:
        sdcard_status = os.statvfs("/sd")
        sdf_block_size = sdcard_status[0]
        sdf_blocks_avail = sdcard_status[4]
        storage_free_percent = sdf_blocks_avail/sdf_block_size *100
        #print( sdcard_status )
        #print( "bytes per block = {}".format(sdf_block_size))
        #print( "free blocks = {}".format(sdf_blocks_avail))
        sd_bytes_avail_B = sdf_blocks_avail * sdf_block_size
        sd_bytes_avail_MB = sd_bytes_avail_B/ 1000000
        #print( "MB available = {}".format(sd_bytes_avail_MB))
        sdfssize = sdcard_status[2]
        sdbytessize_MB = int (round(( sdfssize * sdf_block_size/ 1000000 ), 0))
        #print( "MB drive size = {}".format(sdbytessize_MB))
        sdbytessize_GB = int( round( sdbytessize_MB /1000, 0 ))
        sdavail_percent = int( sd_bytes_avail_MB/ sdbytessize_MB * 100)
        if verbose:
            if sdbytessize_GB < 1:
                print( "SD card space available = {} % of {} MB".format(sdavail_percent, sdbytessize_MB))
            else:
                print( "SD card space available = {} % of {} GB".format(sdavail_percent, sdbytessize_GB))
        if False:
            line_bytes_size = 200 #bytes_per_line
            line_capacity_remaining = int(sd_bytes_avail_MB * 1000000/ line_bytes_size)
            sampling_interval_s = 1.0
            lines_per_s= 1/sampling_interval_s
            time_remaining_h = int( line_capacity_remaining * lines_per_s /3600 )
        time_remaining_h = sd_bytes_avail_B/ bytes_per_hour
        time_remaining_days = round(time_remaining_h/24, 1)
        if verbose:
            print( "data collection time remaining until this SD card is full: {} h = {} days".format(time_remaining_h, time_remaining_days))
    except Exception as err:
        print( err )
        time_remaining_h = False
    return time_remaining_h

def initialize_camera():
    """Initialize the OV2640 camera without the Arducam driver's infinite detection loops."""
    try:
        camera = ArducamClass(OV2640)

        # OV2640 identity check. 0x41 is the revision returned by the tested module.
        camera.I2cAddress = 0x30
        camera.wrSensorReg8_8(0xff, 0x01)
        id_h = camera.rdSensorReg8_8(0x0a)
        id_l = camera.rdSensorReg8_8(0x0b)
        print("OV2640 ID: 0x{:02x} 0x{:02x}".format(id_h, id_l))
        if not (id_h == 0x26 and id_l in (0x40, 0x41, 0x42)):
            print("camera not detected; Q2 will continue without images")
            return False

        # ArduChip SPI loopback test.
        camera.Spi_write(0x00, 0x56)
        if camera.Spi_read(0x00)[0] != 0x56:
            print("camera SPI interface failed; Q2 will continue without images")
            return False

        camera.set_format(JPEG)
        camera.Camera_Init()
        camera.OV2640_set_JPEG_size(OV2640_320x240)
        time.sleep(1)
        camera.clear_fifo_flag()
        print("OV2640 camera ready at 320x240 JPEG")
        return camera
    except Exception as err:
        print("camera initialization failed: {}".format(err))
        print("Q2 will continue without images")
        return False


def make_camera_filename(iso8601_utc_timestamp, batch_number, measurement_number):
    try:
        batch_int = int(batch_number)
    except (TypeError, ValueError):
        batch_int = 0
    return "Q2_{}_B{:03}_M{:02}.jpg".format(
        iso8601_utc_timestamp, batch_int, int(measurement_number)
    )


def capture_camera_frame(camera):
    """Trigger one JPEG and leave it in the Arducam FIFO. Returns byte length or False."""
    if not camera:
        return False
    try:
        camera.flush_fifo()
        camera.clear_fifo_flag()
        camera.start_capture()
        capture_start = time.monotonic()
        while not camera.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK):
            if time.monotonic() - capture_start > CAMERA_CAPTURE_TIMEOUT_S:
                print("camera capture timeout")
                camera.clear_fifo_flag()
                return False
            time.sleep(0.005)

        image_length = camera.read_fifo_length()
        if image_length <= 0 or image_length > MAX_FIFO_SIZE:
            print("camera returned invalid JPEG length: {}".format(image_length))
            camera.clear_fifo_flag()
            return False
        return image_length
    except Exception as err:
        print("camera capture failed: {}".format(err))
        try:
            camera.clear_fifo_flag()
        except Exception:
            pass
        return False


def save_camera_fifo_to_sd(camera, image_path, image_length, buffer):
    """Copy the captured JPEG from the Arducam FIFO to the already-mounted Q2 SD card."""
    if not camera or not image_length or buffer is None:
        return False
    try:
        remaining = image_length
        with open(image_path, "wb") as image_file:
            camera.SPI_CS_LOW()
            try:
                camera.set_fifo_burst()
                while remaining > 0:
                    amount = min(len(buffer), remaining)
                    camera.spi.readinto(buffer, start=0, end=amount)
                    image_file.write(buffer[:amount])
                    remaining -= amount
            finally:
                camera.SPI_CS_HIGH()

        camera.clear_fifo_flag()
        print("image saved: {} ({} bytes)".format(image_path, image_length))
        return True
    except Exception as err:
        print("camera image save failed: {}".format(err))
        try:
            camera.SPI_CS_HIGH()
        except Exception:
            pass
        try:
            camera.clear_fifo_flag()
        except Exception:
            pass
        return False


def initialize_sd_card():
    try:
        sd_cs = digitalio.DigitalInOut(board.SD_CS)
        sd_spi = busio.SPI(board.SD_SCK, board.SD_MOSI, board.SD_MISO)
        sdcard = adafruit_sdcard.SDCard(sd_spi, sd_cs)
        vfs = storage.VfsFat(sdcard)
        storage.mount(vfs, "/sd")
        print( "SD card success" )
    except Exception as err:
        print("SD card fail: missing or full: {}".format(err))
        print( "*** The card may be missing, or you may need to create a folder named sd in the root directory of CIRCUITPY ***" )
        vfs = False
    return vfs

def get_uid():
    try:
        UID = int.from_bytes(microcontroller.cpu.uid, "big") % 10000
        print("unique identifier (UID) : {0}".format( UID ))
    except:
        UID = False
        print("unique identifier (UID) not available")
    return UID

def memory_check( message ):
    gc.collect()
    mem_free_kB = gc.mem_free()/1000
    print( "{} memory free: {} kB, {} %".format( message, int(mem_free_kB), int((100* (mem_free_kB)/start_mem_free_kB ))))

def stall():
    print("intentionally stalled, press return to continue")
    input_string = False
    while input_string == False:
        input_string = input().strip()

gc.collect()
print( "memory free after function definitions = {} kB, {} %".format(int(gc.mem_free()/1000), int(100*(gc.mem_free()/1000)/start_mem_free_kB )) )

main()
