/*
 OV7670 driver for esp32 camera application.

 samples taken from Linus Torvals Linux, Omnivision Implementation Guide from 2005.
 OV7670 Datasheet from 2005.

 Thomas Krueger, Hofgeismar Germany
 Mai 2020.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "sccb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor.h"
#include "esp_log.h"

static const char *TAG = "OV7670";


struct regval_list
{
    unsigned char reg_num;
    unsigned char value;
};


/*

 * The default register settings, as obtained from OmniVision.  There

 * is really no making sense of most of these - lots of "reserved" values

 * and such.

 *
A bug in the ov7670 sensor causes it to introduce noise unless the CLKRC
register is rewritten *after* setting the image mode.  Naturally,
resetting CLKRC in this way will cause other modes to fail.  So
carefully poke the register only when indicated.

 * These settings give VGA YUYV.  TomK

 */
static struct regval_list ov7670_default_regs[] =
{
//   { 0x12, 0x80 }, //Software Reset TomK, done below with delay
    /*  at 24mhz xclk:

     * Clock scale: 3 = 15fps

     *              2 = 20fps

     *              1 = 30fps

     */
    { 0x11, 0x01 }, /*CLKRC: DIV 2: xclk clock prescaler TomK    7=0(no doubleclock);6=0(no external clock); 5-0:0x05=divider+1*/
    { 0x3a, 0x04 }, /* TSLB , TomK bit3=0=yuyv*/
    { 0x12, 0 }, /* COM7 = default value=0 YUV*/
    /*

     * Set the hardware window.  These values from OV don't entirely

     * make sense - hstop is less than hstart.  But they work...

     */
    { 0x17, 0x13 },  //Hstart: 0x13+HREF[2-0]0x06 = 158
    { 0x18, 0x01 },  //Hstop: 0x01+HREF[5-3]0x06 = 14
    { 0x32, 0xb6 }, //HREF
    { 0x19, 0x02 }, //Vstart: 0x02+VREF[1-0]0x02 = 10
    { 0x1a, 0x7a }, //Vstop:  0x7a+VREF[3-2]0x02 = 490
    { 0x03, 0x0a }, //VREF
    { 0x0c, 0 }, // disable scaling
    { 0x3e, 0 }, // disable scaling
	
    /* Mystery scaling numbers */
    { 0x70, 0x3a },
    { 0x71, 0x35 },
    { 0x72, 0x11 },
    { 0x73, 0xf0 },
    { 0xa2, 0x02 },
    { 0x15, 0x22 }, //0x02vsync negative. pclk pause on blank TomK
    /* Gamma curve values */
    { 0x7a, 0x20 },
    { 0x7b, 0x10 },
    { 0x7c, 0x1e },
    { 0x7d, 0x35 },
    { 0x7e, 0x5a },
    { 0x7f, 0x69 },
    { 0x80, 0x76 },
    { 0x81, 0x80 },
    { 0x82, 0x88 },
    { 0x83, 0x8f },
    { 0x84, 0x96 },
    { 0x85, 0xa3 },
    { 0x86, 0xaf },
    { 0x87, 0xc4 },
    { 0x88, 0xd7 },
    { 0x89, 0xe8 },
    /* AGC and AEC parameters.  Note we start by disabling those features,

       then turn them only after tweaking the values. */
    { 0x13, 0x80 | 0x40 | 0x20 }, // 7=fast AGC/AEC; 6=AEC step unlimit; 5=banding filter off; 2=AGC on; 1=AWB on; 0=AEC 0n
    { 0x00, 0 },
    { 0x10, 0 },
    { 0x0d, 0x40 }, /* magic reserved bit */
    { 0x14, 0x18 }, /* 4x gain + magic rsvd bit */
    { 0xa5, 0x05 },
    { 0xab, 0x07 },
    { 0x24, 0x95 },
    { 0x25, 0x33 },
    { 0x26, 0xe3 },
    { 0x9f, 0x78 },
    { 0xa0, 0x68 },
    { 0xa1, 0x03 }, /* magic */
    { 0xa6, 0xd8 },
    { 0xa7, 0xd8 },
    { 0xa8, 0xf0 },
    { 0xa9, 0x90 },
    { 0xaa, 0x94 },
    { 0x13, 0x80|0x40|0x20|0x04|0x01 }, // 7=fast AGC/AEC; 6=AEC step unlimit; 5=banding filter off; 2=AGC on; 1=AWB on; 0=AEC 0n
    /* Almost all of these are magic "reserved" values.  */
    { 0x0e, 0x61 },
    { 0x0f, 0x4b },
    { 0x16, 0x02 },
    { 0x1e, 0x07 },
    { 0x21, 0x02 },
    { 0x22, 0x91 },
    { 0x29, 0x07 },
    { 0x33, 0x0b },
    { 0x35, 0x0b },
    { 0x37, 0x1d },
    { 0x38, 0x71 },
    { 0x39, 0x2a },
    { 0x3c, 0x78 },
    { 0x4d, 0x40 },
    { 0x4e, 0x20 },
    { 0x69, 0 },
    { 0x6b, 0x4a },
    { 0x74, 0x10 },
    { 0x8d, 0x4f },
    { 0x8e, 0 },
    { 0x8f, 0 },
    { 0x90, 0 },
    { 0x91, 0 },
    { 0x96, 0 },
    { 0x9a, 0 },
    { 0xb0, 0x84 },
    { 0xb1, 0x0c }, // black level auto
    { 0xb2, 0x0e },
    { 0xb3, 0x82 },
    { 0xb8, 0x0a },
    /* More reserved magic, some of which tweaks white balance */
    { 0x43, 0x0a },
    { 0x44, 0xf0 },
    { 0x45, 0x34 },
    { 0x46, 0x58 },
    { 0x47, 0x28 },
    { 0x48, 0x3a },
    { 0x59, 0x88 },
    { 0x5a, 0x88 },
    { 0x5b, 0x44 },
    { 0x5c, 0x67 },
    { 0x5d, 0x49 },
    { 0x5e, 0x0e },
    { 0x6c, 0x0a },
    { 0x6d, 0x55 },
    { 0x6e, 0x11 },
    { 0x6f, 0x9f }, /* "9e for advance AWB" */
    { 0x6a, 0x40 },
    { 0x01, 0x40 },
    { 0x02, 0x60 },
    { 0x13, 0x80|0x40|0x20|0x04|0x01|0x02 }, // 7=fast AGC/AEC; 6=AEC step unlimit; 5=banding filter off; 2=AGC on; 1=AWB on; 0=AEC 0n
    /* Matrix coefficients */
    { 0x4f, 0x80 },
    { 0x50, 0x80 },
    { 0x51, 0 },
    { 0x52, 0x22 },
    { 0x53, 0x5e },
    { 0x54, 0x80 },
    { 0x58, 0x9e }, // auto contrast center
    { 0x41, 0x08 },
    { 0x3f, 0 },
    { 0x75, 0x05 },
    { 0x76, 0xe1 }, // pixel correction
    { 0x4c, 0 },
    { 0x77, 0x01 },
    { 0x3d, 0xc1 }, // bit1=0=yuyv, was 0xc3 TomK
    { 0x4b, 0x09 },
    { 0xc9, 0x60 }, // UV saturation minimum
    { 0x41, 0x38 },  // auto edge enhance, auto denoise
    { 0x56, 0x40 },
    { 0x34, 0x11 },
    { 0x3b, 0x02|0x10 }, //enable 50hz auto
    { 0xa4, 0x88 },
    { 0x96, 0 },
    { 0x97, 0x30 },
    { 0x98, 0x20 },
    { 0x99, 0x30 },
    { 0x9a, 0x84 },
    { 0x9b, 0x29 },
    { 0x9c, 0x03 },
    { 0x9d, 0x4c },
    { 0x9e, 0x3f },
    { 0x78, 0x04 },
    /* Extra-weird stuff.  Some sort of multiplexor register */
    { 0x79, 0x01 },
    { 0xc8, 0xf0 },
    { 0x79, 0x0f },
    { 0xc8, 0x00 },
    { 0x79, 0x10 },
    { 0xc8, 0x7e },
    { 0x79, 0x0a },
    { 0xc8, 0x80 },
    { 0x79, 0x0b },
    { 0xc8, 0x01 },
    { 0x79, 0x0c },
    { 0xc8, 0x0f },
    { 0x79, 0x0d },
    { 0xc8, 0x20 },
    { 0x79, 0x09 },
    { 0xc8, 0x80 },
    { 0x79, 0x02 },
    { 0xc8, 0xc0 },
    { 0x79, 0x03 },
    { 0xc8, 0x40 },
    { 0x79, 0x05 },
    { 0xc8, 0x30 },
    { 0x79, 0x26 },
    { 0xff, 0xff }, /* END MARKER */
};


/*

 * Here we'll try to encapsulate the changes for just the output

 * video format.

 *

 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.

 *

 * IMPORTANT RULE: the first entry must be for COM7, see ov7670_s_fmt for why.
 * HACK: if we're running rgb565 we need to grab then rewrite
	 * CLKRC.  If we're *not*, however, then rewriting clkrc hoses
	 * the colors.
 * COM7 is a pain in the ass, it doesn't like to be read then
	 * quickly written afterward.  But we have everything we need
	 * to set it absolutely here, as long as the format-specific
	 * register sets list it first.
 */
static struct regval_list ov7670_fmt_yuv422[] =
{
    { 0x12, 0x0 }, /* Selects YUV mode (COM7)*/
    { 0x8c, 0 }, /* No RGB444 please **first */
    { 0x04, 0 }, /* CCIR601 **first */
    { 0x40, 0xc0 }, //=default. **first

    { 0x14, 0x48 }, /* 32x gain ceiling; 0x8 is reserved bit ***different to 0x18 */
    { 0x4f, 0x80 }, /* "matrix coefficient 1" */
    { 0x50, 0x80 }, /* "matrix coefficient 2" */
    { 0x51, 0 }, /* vb */
    { 0x52, 0x22 }, /* "matrix coefficient 4" */
    { 0x53, 0x5e }, /* "matrix coefficient 5" */
    { 0x54, 0x80 }, /* "matrix coefficient 6"  */
    { 0x3d, 0xC1 }, // bit1=0=yuyv  to 0xC1. auto saturation uv
    { 0xff, 0xff },
};


static struct regval_list ov7670_fmt_rgb565[] =
{
    { 0x12, 0x04 }, // Selects RGB mode rgb565(COM7)
    { 0x8c, 0 }, // No RGB444 please
    { 0x04, 0x0 }, // CCIR601
    { 0x40, 0x10 },
    { 0x14, 0x38 }, // 16x gain ceiling; 0x8 is reserved bit
    { 0x4f, 0xb3 }, // "matrix coefficient 1"
    { 0x50, 0xb3 }, // "matrix coefficient 2"
    { 0x51, 0 }, // vb
    { 0x52, 0x3d }, // "matrix coefficient 4"
    { 0x53, 0xa7 }, // "matrix coefficient 5"
    { 0x54, 0xe4 }, // "matrix coefficient 6"
    { 0x3d, 0x80|0x40 }, //gamma, uv-auto 
    { 0xff, 0xff },
};

/* The digital image scaler can be used to reduce the imagesize.(crop/field of view/zoom)

There are 3 predefined scaler settings in COM7 for qvga,cir,qcif or you can set the scale to your gusto.
manual scale setting:(raw sensor array is: 656x488)
0x3e[3]=1; // enable scaler
0xa2[7]=1; // enable pclk delay ??
Then set your output windowsize:
hstart = 0x17[7-0] = (hstart >> 3); 0x32[2-0]= (hstart & 0x07); //hstart/stop= 11Bits (2040 max). defines length of HREF puls
hstop =  0x18[7-0] = (hstop >> 3);  0x32[5-3]= ((hstop & 0x07) << 3)
vstart = 0x19[7-0] = (vstart >> 2); 0x03[1-0]= (vstart & 0x02); // vstart/stop = 10Bits (1020 max). defines length of VREF puls
vstop =  0x1a[7-0] = (vstop >> 2);  0x03[13-2]= ((vstop & 0x02) << 2);

*/
//standard
static struct regval_list format_vga[] =
{

    {0x0c, 0x00}, // COM3 DCW/Downsampling[2] and scale/zoom[3] disable
    {0x3e, 0x00}, // scaling and dcw pclk adjust = off
    {0x70, 0x3a}, // [7]=testpatterh; hor. scale ratio[6-0]=58 same. Value=0x20 * Imagesize(downsampled) / NewImageSize. (Zoomfactor)
    {0x71, 0x35}, // [7]=testpatterh; vertial scale ratio[6-0]=53 same
    {0x72, 0x11}, // DCW/Downsampling mode: downsamplingHV 2x2 = reduce size by 4
    {0x73, 0xf0}, // pclk div dsp scale output
    {0xa2, 0x02}, // scaling pclk delay : = Original Hsize/pixleclockdivider - New Hsize
	
    {0xff, 0xff},// end marker!!
};

//standard
static struct regval_list format_qvga[] =
{
//http://mcunovel.blogspot.com/2013/07/get-image-from-ov7670-with-various.html	


    {0x0c, 0x04},  // DCW/Downsampling on
    {0x3e, 0x19},  //scaling and dcw pclk-scale div by 8
    {0x70, 0x3a},  // hor. scale ratio[6-0]=58=0,5
    {0x71, 0x35},  // vertial scale ratio[6-0]=53=0,5
    {0x72, 0x11},  // DCW/Downsampling mode: downsamplingHV 2x2 = reduce size by 4
    {0x73, 0xf1},   // ScaleOut clock divider on, divide by 2
    {0xa2, 0x02},  // scaling pclk delay = 2clocks

    {0xff, 0xff},// end marker!!
};

//custom
static struct regval_list format_qqvga[] =
{
    {0x0c, 0x04}, // DCW/Downsampling on 
    {0x3e, 0x1a}, //scaling and dcw pclk-scale div by 4 
    {0x70, 0x3a},  // hor. scale ratio[6-0]=58=0,5
    {0x71, 0x35},   // vertial scale ratio[6-0]=53=0,5
    {0x72, 0x22},   // DCW/Downsampling mode: downsamplingHV 4x4 = reduce size by 16
    {0x73, 0xf2},   // ScaleOut clock divider oon, divide by 4
    {0xa2, 0x02},  // scaling pclk delay = 2clocks
	
    {0xff, 0xff},// end marker!!
};

//standard.  352 * 240, not 400 * 292 !!
static struct regval_list format_cif[] =
{
    {0x0c, 0x08}, // scale/zoom on 
    {0x3e, 0x11}, //dcw pclk-scale div by 2
    {0x70, 0x3a},  // hor. scale ratio[6-0]=58=0,5
    {0x71, 0x35},   // vertial scale ratio[6-0]=53=0,5
    {0x72, 0x11},   // DCW/Downsampling mode: downsamplingHV 2x2 = reduce size by 4
    {0x73, 0xf1},  // ScaleOut clock divider on, divide by 2
    {0xa2, 0x02},  // scaling pclk delay = 2clocks
	
    {0xff, 0xff},// end marker!!
};

// standard
static struct regval_list format_qcif[] =
{
    {0x0c, 0x0c}, // DCW/Downsampling AND scale/zoom on 
    {0x3e, 0x11}, //dcw pclk-scale div by 2
    {0x70, 0x3a},  // hor. scale ratio[6-0]=58=0,5
    {0x71, 0x35},  // vertial scale ratio[6-0]=53=0,5
    {0x72, 0x11},  // DCW/Downsampling mode: downsamplingHV 2x2 = reduce size by 4
    {0x73, 0xf1},  // ScaleOut clock divider on, divide by 2
    {0xa2, 0x52},  // scaling pclk delay = 82 clocks
	
    {0xff, 0xff},// end marker!!
};






static int write_reglist(sensor_t *sensor, struct regval_list *vals)
{
    int ret = 0;

    while ( (vals->reg_num != 0xff || vals->value != 0xff) && (ret == 0) )
    {
        ret = SCCB_Write(sensor->slv_addr, vals->reg_num, vals->value);


        vals++;
    }

    return ret;
}

// digital crob the image to a defined size (zooming in). adjust to remove dummy lines/coloms
static int ov7670_frame_control(sensor_t *sensor, int hstart, int hstop, int vstart, int vstop) //158(0x9e), 14(0x0e), 10(0x0a), 490(0x1e8));
{

struct regval_list frame[7];

    frame[0].reg_num = 0x17;  //0x17  hstart = hor.start position. HREF puls is streched.
    frame[0].value = (hstart >> 3);

    frame[1].reg_num = 0x18; //0x18  hstop = hor.end position. HREF puls is cut.
    frame[1].value = (hstop >> 3);

    frame[2].reg_num = 0x32; //0x32 HREF contains lower 3 bits of hstart/hstop
    frame[2].value = (((hstop & 0x07) << 3) | (hstart & 0x07));
    
    frame[3].reg_num = 0x19; //0x19  vertical start row.  VSYNC should be strectched.
    frame[3].value = (vstart >> 2);
    
    frame[4].reg_num = 0x1a; //0x1a  vertical end row. VSYNC should be cut
    frame[4].value = (vstop >> 2);

    frame[5].reg_num = 0x03; //0x03  VREF contains lower 2 bits of vstart and vstop
    frame[5].value = (((vstop & 0x02) << 2) | (vstart & 0x02));

    /* End mark */
    frame[5].reg_num = 0xFF;
    frame[5].value = 0xFF;

    return write_reglist(sensor, frame);
}


static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
	
uint8_t reg;

	reg = SCCB_Read(sensor->slv_addr, 0x11); // save CLKRC

    switch (pixformat) {
        case PIXFORMAT_RGB565:
        case PIXFORMAT_RGB888:
            write_reglist(sensor, ov7670_fmt_rgb565);
        break;
 
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
	    default:
            write_reglist(sensor, ov7670_fmt_yuv422);
        break;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    /*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 */
        SCCB_Write(sensor->slv_addr, 0x11, reg); 
	ESP_LOGI(TAG, "set pixformat: %d",pixformat);
    sensor->pixformat = pixformat;

    return 0;
}

// (raw sensor array is: 656x488)
static int set_framesize(sensor_t *sensor, framesize_t framesize)
{

	switch (framesize)
	{
		case FRAMESIZE_VGA: //standard 640 480
		write_reglist(sensor, format_vga);
		ov7670_frame_control(sensor, 158, 14, 10, 490); // hstart,hstop,vstart,vstop. 784=virtual length. 158+640-784=hstop
		break;
		case FRAMESIZE_CIF: //standard 352x240 downsampling 2x2. 
		write_reglist(sensor, format_cif);
		ov7670_frame_control(sensor, 170, 90, 14, 494);//fail //linelength =784-hstart+hstop. hstart=784+hstop-linelength
		break;
		case FRAMESIZE_QVGA: //standard 320X240 downsampling 2x2. 4 times smaler
		write_reglist(sensor, format_qvga);
		ov7670_frame_control(sensor, 168, 24, 12, 492);//ok
		break;
		case FRAMESIZE_QCIF: //standard 176x144 downsampling 2x2
		write_reglist(sensor, format_qcif);
		ov7670_frame_control(sensor, 456, 24, 14, 494); //ok
		break;
		case FRAMESIZE_QQVGA: //custom 160x120  downsampling 4x4. 16 times smaller
		write_reglist(sensor, format_qqvga);
		ov7670_frame_control(sensor, 168, 24, 12, 492); //fail
		break;
		default:
		return 1;
	}
	

	sensor->status.framesize = framesize; // tell camera.c driver about new resolution
	    vTaskDelay(40 / portTICK_PERIOD_MS);
	
	ESP_LOGI(TAG, "set framesize: %d",framesize);
    return 0;
}

static int init_status(sensor_t *sensor)
{
    sensor->status.awb_gain = 1;
    sensor->status.aec = 1;
    sensor->status.agc = 1;
    sensor->status.raw_gma = 1;
    sensor->status.lenc = 1;
    sensor->status.hmirror = 0;
    sensor->status.vflip = 0;
    sensor->status.colorbar = 0;
    sensor->status.quality = 50;
	//ESP_LOGI(TAG, "init_status called!");
    return 0;
}

// maximum agc value: range 2 to 128 in binary steps power 2.
// expect input range: 0 - 6
static int set_gainceiling(sensor_t *sensor, gainceiling_t val)
{
    uint8_t x,b;
    b = SCCB_Read(sensor->slv_addr, 0x14);
    b&=0x8f;
	x=val;
	x<<=4;
	x&=0x70;
    b|=x;
    SCCB_Write(sensor->slv_addr, 0x14,b);
    //ESP_LOGI(TAG, "set_gain ceiling to:%u reg:0x%02x",x,b);

    return 0;
}


// set auto agc
static int set_gain_ctrl(sensor_t *sensor, int enable) //auto gain
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x13);

    if (enable)
    {
        val|=0x04;
    }
    else
    {
        val&=0xfb;
    }
    SCCB_Write(sensor->slv_addr, 0x13, val);
    //ESP_LOGI(TAG, "set_gain_ctrl");

    return 0;
}


// set the manual gain =10bit value (1023)
// input range from webpage: 0 - 30. complicated formular
static  int set_agc_gain(sensor_t *sensor, int value)
{
    uint16_t val;
    uint8_t x;
    val=value;
	if (val>15)
	{
		val<<=4;
	}
    x=val;
    SCCB_Write(sensor->slv_addr, 0x00, x); //lower 8 bits
    x=SCCB_Read(sensor->slv_addr, 0x03);
    val>>=8;
    val&=0x03;
    val<<=6;
    x&=0x3f;
    x|=val;
    SCCB_Write(sensor->slv_addr, 0x03, x); //upper 2 bits
    //ESP_LOGI(TAG, "set_agc_gain");

    return 0;

}

/*
//0=bypass awb gain; 1= enable awb gain. we leave it on, it messes up the colors
static int set_awb_gain(sensor_t *sensor, int enable)
{
	uint8_t val = SCCB_Read(sensor->slv_addr, 0x41);

    if (enable)
	{
        val|=0x08;
	}
    else
	{
        val&=0xf7;
	}
        SCCB_Write(sensor->slv_addr, 0x41, val);
    //ESP_LOGI(TAG, "set awb gain");

    return 0;
}
*/

static int set_autoawb(sensor_t *sensor, int enable) // set auto awb whitebalance
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x13);

    if (enable)
    {
        val|=0x02; //auto
    }
    else
    {
        val&=0xfd; //manual
    }
    SCCB_Write(sensor->slv_addr, 0x13, val);
    //ESP_LOGI(TAG, "set_autoawb");

    return 0;
}


// advanced whitebalance(0) based on color temperatur or normal mode(1)= average red,green,blue
static int set_awb_advanced(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x6f);

    if (!enable) //normal
    {
        val|=0x01;
    }
    else
    {
        val&=0xfe;
    }
    SCCB_Write(sensor->slv_addr, 0x6f, val);
    //ESP_LOGI(TAG, "set_awb_advanced");

    return 0;
}

// awb gain
static int set_awb_gain(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x41);

    if (!enable) //on
    {
        val|=0x04;
    }
    else
    {
        val&=0xf7;
    }
    SCCB_Write(sensor->slv_addr, 0x41, val);
    //ESP_LOGI(TAG, "set_awb_advanced");

    return 0;
}



// white pixel correct
static int set_wpc(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x76);

    if (!enable) //on
    {
        val|=0x40;
    }
    else
    {
        val&=0xbf;
    }
    SCCB_Write(sensor->slv_addr, 0x76, val);
 

    return 0;
}



static int set_exposure_ctrl(sensor_t *sensor, int enable) //auto exposure on off
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x13);

    if (enable)
    {
        val|=0x01;
    }
    else
    {
        val&=0xfe;
    }
    SCCB_Write(sensor->slv_addr, 0x13, val);
    //ESP_LOGI(TAG, "set_exposure_ctrl");

    return 0;
}

// set a 16bit exposure value: 0x07[5-0]+0x10[7-0]+0x04[1-0]
static int set_aec_value(sensor_t *sensor, int value) //manual exposure value
{
    uint8_t x,y;
   // ESP_LOGI(TAG, "set_aec_value:%d",value);

    x = SCCB_Read(sensor->slv_addr, 0x04);
    y=value&0x03;
    x&=0xFC;
    x|=y;
    SCCB_Write(sensor->slv_addr, 0x04, x);

    value>>=2;
    x=value;
    SCCB_Write(sensor->slv_addr, 0x10, x);

    x = SCCB_Read(sensor->slv_addr, 0x07);
    value>>=8;
    y=value&0x3f;
    x&=0xC0;
    x|=y;
    SCCB_Write(sensor->slv_addr, 0x07, x);

    return 0;
}


static int set_gamma(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x3d);

    if (enable)
    {
        val|=0x81;
    }
    else
    {
        val&=0x7e;
    }
    SCCB_Write(sensor->slv_addr, 0x3d, val);

    return 0;
}

static int set_lenscorrection(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x66);

    if (enable)
    {
        val|=0x01;
    }
    else
    {
        val&=0xfe;
    }
    SCCB_Write(sensor->slv_addr, 0x66, val);

    return 0;
}



static int set_uv_adjust(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x3d);

    if (enable)
    {
        val|=0x40;
    }
    else
    {
        val&=0xbf;
    }
    SCCB_Write(sensor->slv_addr, 0x3d, val);

    return 0;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x71);

    if (enable)
    {
        val|=0x80; //normal color bar
    }
    else
    {
        val&=0x7f;
    }
// 0x70,0x71 select color bar type: normal bar=0x71=1,0x70=0. just this !!!
// 0x42 = digital color bar. this gives a not nice colorbar.
// 0x12 gives overlay colorbar
    SCCB_Write(sensor->slv_addr, 0x71, val);

    return 0;
}




static int set_hmirror(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x1e);

    if (enable)
    {
        val|=0x20;
    }
    else
    {
        val&=0xdf;
    }
    SCCB_Write(sensor->slv_addr, 0x1e, val);

    return 0;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x1e);

    if (enable)
    {
        val|=0x10;
    }
    else
    {
        val&=0xef;
    }
    SCCB_Write(sensor->slv_addr, 0x1e, val);

    return 0;

}
static  int set_special_effect(sensor_t *sensor, int effect)
{


    switch (effect)
    {
    case 1: //negative
        SCCB_Write(sensor->slv_addr, 0x3a, 0x24);
        SCCB_Write(sensor->slv_addr, 0x67, 0x80);
        SCCB_Write(sensor->slv_addr, 0x68, 0x80);
        break;
    case 2: //grayscale
        SCCB_Write(sensor->slv_addr, 0x3a, 0x14);
        SCCB_Write(sensor->slv_addr, 0x67, 0x80);
        SCCB_Write(sensor->slv_addr, 0x68, 0x80);
        break;
    case 3://red tint
        SCCB_Write(sensor->slv_addr, 0x3a, 0x14);
        SCCB_Write(sensor->slv_addr, 0x67, 0xc0);
        SCCB_Write(sensor->slv_addr, 0x68, 0x80);
        break;
    case 4://green tint
        SCCB_Write(sensor->slv_addr, 0x3a, 0x14);
        SCCB_Write(sensor->slv_addr, 0x67, 0x40);
        SCCB_Write(sensor->slv_addr, 0x68, 0x40);
        break;
    case 5://blue tint
        SCCB_Write(sensor->slv_addr, 0x3a, 0x14);
        SCCB_Write(sensor->slv_addr, 0x67, 0x80);
        SCCB_Write(sensor->slv_addr, 0x68, 0xc0);
        break;
    case 6://sepia/negative
        SCCB_Write(sensor->slv_addr, 0x3a, 0x24);
        SCCB_Write(sensor->slv_addr, 0x67, 0x80);
        SCCB_Write(sensor->slv_addr, 0x68, 0x80);
        break;
    case 7: //antique
        SCCB_Write(sensor->slv_addr, 0x3a, 0x14);
        SCCB_Write(sensor->slv_addr, 0x67, 0xa0);
        SCCB_Write(sensor->slv_addr, 0x68, 0x40);
        break;
    case 0: // no effect
    default:
        SCCB_Write(sensor->slv_addr, 0x3a, 0x04);
        SCCB_Write(sensor->slv_addr, 0x67, 0xc0);
        SCCB_Write(sensor->slv_addr, 0x68, 0x80);
        break;
    }
    //ESP_LOGI(TAG, "set_special_effect:%d",effect);
    return 0;
}


static  int set_wb_mode(sensor_t *sensor, int mode)
{

//set_autoawb(sensor,0); // auto off

    switch (mode)
    {
    case 1: //sunny
        SCCB_Write(sensor->slv_addr, 0x01, 0x5a); //BLUE
        SCCB_Write(sensor->slv_addr, 0x02, 0x5c); //RED
        break;
    case 2: //cloudy
        SCCB_Write(sensor->slv_addr, 0x01, 0x58);
        SCCB_Write(sensor->slv_addr, 0x02, 0x60);
        break;
    case 3: //office
        SCCB_Write(sensor->slv_addr, 0x01, 0x84);
        SCCB_Write(sensor->slv_addr, 0x02, 0x4c);
        break;
    case 4://home
        SCCB_Write(sensor->slv_addr, 0x01, 0x96);
        SCCB_Write(sensor->slv_addr, 0x02, 0x40);
        break;
    case 0: //auto
    default:
        SCCB_Write(sensor->slv_addr, 0x01, 0x80);
        SCCB_Write(sensor->slv_addr, 0x02, 0x80);
        break;
    }


    //ESP_LOGI(TAG, "set_wb_mode:%d",mode);

    return 0;
}

static  int set_contrast(sensor_t *sensor, int level)
{
    uint8_t val;
    switch (level)
    {
    case 3:
        val=0x70;
        break;
    case 2:
        val=0x60;
        break;
    case 1:
        val=0x50;
        break;
    case -1:
        val=0x30;
        break;
    case -2:
        val=0x20;
        break;
    case -3:
        val=0x10;
        break;
    default:
        val=0x40;
        break;
    }

    SCCB_Write(sensor->slv_addr, 0x56, val);
    //ESP_LOGI(TAG, "set contrast to:%d",level);

    return 0;

}

//bit7=sign bit. 1=negativ 0=positiv change
static  int set_brightness(sensor_t *sensor, int level)
{
    uint8_t val;
    switch (level)
    {
    case 3:
        val = 0x48;
        break;
    case 2:
        val=0x30;
        break;
    case 1:
        val=0x18;
        break;
    case -1:
        val=0x98;
        break;
    case -2:
        val=0xb0;
        break;
    case -3:
        val=0xc8;
        break;
    case 0:
    default:
        val=0x00;
        break;
    }

    SCCB_Write(sensor->slv_addr, 0x55, val);
    //ESP_LOGI(TAG, "set brightness to:%d",level);

    return 0;

}
static  int set_saturation(sensor_t *sensor, int level)
{


    //color matrix values
    uint16_t w;
    w = 0x80 + 0x20 * level;
    if (w>0xff) w=0xff;
    SCCB_Write(sensor->slv_addr, 0x4f, w);
    SCCB_Write(sensor->slv_addr, 0x50, w);
    SCCB_Write(sensor->slv_addr, 0x54, w);
    w = 0x22 + (0x11 * level) / 2;
    if (w>0xff) w=0xff;
    SCCB_Write(sensor->slv_addr, 0x52, w);
    w = 0x5e + (0x2f * level) / 2;
    if (w>0xff) w=0xff;
    SCCB_Write(sensor->slv_addr, 0x53, w);

    SCCB_Write(sensor->slv_addr, 0x51, 0x00);
    SCCB_Write(sensor->slv_addr, 0x58, 0x9e);  //matrix signs

    //ESP_LOGI(TAG, "set saturation to:%d",level);

    return 0;

}



static int set_nightmode(sensor_t *sensor, int enable) //enable nightmode
{
    uint8_t val = SCCB_Read(sensor->slv_addr, 0x3b);

    if (enable)
    {
        val|=0xe0;
    }
    else
    {
        val&=0x1f;
    }
    SCCB_Write(sensor->slv_addr, 0x3b, val);
    //ESP_LOGI(TAG, "set_nightmode");

    return 0;
}


static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    SCCB_Write(sensor->slv_addr, reg, value);
    //ESP_LOGI(TAG, "Set Reg: 0x%02x Mask:0x%02x Value:0x%02x", reg,mask,value);
    return 0;
}

static int  get_reg(sensor_t *sensor, int reg, int mask)
{
    //ESP_LOGI(TAG, "Get Reg: 0x%02x Mask:0x%02x", reg,mask);
    return(SCCB_Read(sensor->slv_addr, reg));
}




/*
Here we fill the sensor-struct function-pointers with this cameras supported ones.
unsupported functionpointers will be left NULL!
its expected the struct to be all 0.

All functions return 0 on success, 1 or -1 on error.

See sensor.h for all available functions..and there definitions!!
*/
int ov7670_init(sensor_t *sensor)
{
    // Set function pointers
    sensor->set_pixformat = set_pixformat;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_colorbar = set_colorbar;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;
    sensor->set_contrast= set_contrast;
    sensor->set_brightness= set_brightness;
    sensor->set_saturation= set_saturation;
    sensor->set_wb_mode=set_wb_mode;
    sensor->set_special_effect = set_special_effect;
	sensor->set_wpc = set_wpc;
    sensor->set_gain_ctrl = set_gain_ctrl;
	sensor->set_awb_gain = set_awb_gain;
    sensor->set_agc_gain = set_agc_gain;
    sensor->set_gainceiling = set_gainceiling;
    sensor->set_whitebal = set_autoawb;  // auto awb
    sensor->set_dcw = set_awb_advanced;
    sensor->set_aec_value = set_aec_value;
    sensor->set_aec2 = set_nightmode;
    sensor->set_raw_gma = set_gamma;
    sensor->set_lenc = set_lenscorrection;
    sensor->set_bpc = set_uv_adjust;
    sensor->get_reg=get_reg;
    sensor->set_reg=set_reg;


    // Retrieve sensor's signature
    sensor->id.MIDH = SCCB_Read(sensor->slv_addr, 0x1c);
    sensor->id.MIDL = SCCB_Read(sensor->slv_addr, 0x1d);
    sensor->id.PID = SCCB_Read(sensor->slv_addr, 0x0a);
    sensor->id.VER = SCCB_Read(sensor->slv_addr, 0x0b);

// final init the camera:
    SCCB_Write(sensor->slv_addr,0x12,0x80);  //reset cam
    vTaskDelay(10 / portTICK_PERIOD_MS);
    /*
            // this minimum config:
            SCCB_Write(sensor->slv_addr, 0x11, 0x05); //5mhz:   7=0;6=0(no doubleclock); 5-0:0x05
            SCCB_Write(sensor->slv_addr, 0x15, 0x22); // vsync-invert, pclk pause on blank
        	SCCB_Write(sensor->slv_addr, 0x3a, 0x04); //Format: reg0x3a[3], reg0x3d[1].YUYV
        	SCCB_Write(sensor->slv_addr, 0x3d, 0x99); //00 = YUYV. seems bit0 changes UV order!! together with bit7(gamma)
    */

    write_reglist(sensor, ov7670_default_regs);
   set_pixformat(sensor,PIXFORMAT_RGB565);


// the I2S can sample at max 20Mhz pclk! otherwise framecapture failes.
//clock : pxclk=inputclock*pll_multiplier / (2x(CLKRC[5:0]+1)). 25mhz*4=100mhz / 10 = 10mhz .(2x(4+1))  from implementation guide!!!
// As we are capturing YUV that needs to be converted to jpg, we do not get higher framerates that 4fps. So let the camera do the same.
    SCCB_Write(sensor->slv_addr, 0x11, 0x00); // input clock=20Mhz. divider 4+1=5; so resluting clock is 20/5= 4Mhz internal clock = Pixelclock
    SCCB_Write(sensor->slv_addr, 0x6b, 0x2a); // no PLL Multiplier. enable internal regulator on bit4=0!!!
    //ESP_LOGI(TAG, "init done");
    return 0;
}


/* Please Note: the "sensor" pointer points to struct sensor_t maintained by the camera.c-driver. Includes all current information.
so updating info here will give info to the driver.
Such as:
    sensor_id_t id;             // Sensor ID.
    uint8_t  slv_addr;          // Sensor I2C slave address.
    pixformat_t pixformat;
    camera_status_t status;
    int xclk_freq_hz;
...list of camera driver functions.
*/
/*
auto settings:
0x13 = AutoGain enable, auto Whitebalance
0x14 = Gain Ceiling=max AGC value allowed
0x3B = Auto 50/60Hz detection
0x3D = UV saturation levels
0x41 = edge enhancement and denoise auto
0x58 = auto contrast center
0x76 = defect pixel correction
0xB1 = auto Black level
0xC9 = auto color saturation register
*/