/* alternate tcp server for camera application

esp32-cam alternate TCP server using lwip directly via BSD Socket API.
To be used for linux motion surveillance applications or similar. NOT for face recoqnition!
OV7670 only!

This file contains:
- camcontrol webserver
  - the onboard LED to be used as flashlight (snapshots) or streaming light.
    This LED draws a higher current and gets quite hot.(no dimming is used)
  - nightmode
  - Status/Framerate display
  - Reset option processor

- camstreaming webserver
  - reduced framerate to balance network load on multible camera usage.(linux motion)
  - BMP header support

NOTES: esp32-cam 5V supply should be increased to min. 5.4V (upto 6V) for stable operation.

september 2020, Thomas Krueger, Hofgeismar Germany (all rights reserved)
*/



#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_log.h"

//protos:
void streamtask(void *param);
int tcpserver(int port);
int http_response(int port, char *req, int connection);
int http_stream(int connection, int convert);
int get_frame(int convert, uint8_t **buf, size_t *len);
static uint16_t set_register(char *uri);
static int set_control(char *uri);
static int get_camstatus(void);
static int get_status(char *uri);
void bmp_mode(int bmp);
void night_mode(int on);
esp_err_t dma_desc_init();

//globals:
camera_fb_t *fb=NULL;
char iobuf[1024]; // for control processing
int flashlight, streamlight, nightmode, IsStreaming, Quality=80;

// jpg stuff
int Convert=0; // jpg conversion enable, if camera is delivering non jpg format. ie. yuv or rgb565

// BMP stuff
void makebmpheader(uint8_t *pbuf, uint16_t width, uint16_t height, uint16_t bpp);
int setbmp(void);
int UseBmp=1; // if set, use bmp format for streaming, else jpg...default on reset
uint8_t BMPhead[100];
#define BMPHDSIZE 68

//framerate stuff
TimerHandle_t tmr;
void timerCallBack( TimerHandle_t xTimer );
int NetFPS,HwFPS, I2sFPS, NetFrameCnt, resetflag=0;
int IRAM_ATTR HwFrameCnt, I2sFrameCnt,CapErrors; // updated in camera.c, line 582

static const char *TAG = "tcpserver";


/*
starts a task for the streamserver on port 81
then goes into control server on port 80 for camera control
*/
void camserver(void)
{
    TaskHandle_t servertask;


	setbmp(); 
	
    //NOTE: the RTOS tick is configured to 10ms. so if you set timerperiod to 1, then its actually 10ms!!
    tmr = xTimerCreate("SecTimer", 100, pdTRUE, (void *)0, &timerCallBack); // create a 1Sec software timer
    xTimerStart(tmr,0);  // and start it

    flashlight=streamlight=IsStreaming=nightmode=0;

// init LED (only for AI-Thinker board!!)
    gpio_set_direction(4, GPIO_MODE_OUTPUT); // set portpin to output. HighPower LED

    // give visual indication that a successfull reset occured by turning LED briefly on
//    gpio_set_level(4, 1); // turn led on
//    vTaskDelay(400/portTICK_PERIOD_MS);  // wait a little to get camera exposure settle to new light conditions
    gpio_set_level(4, 0); // turn led off

// set starting streamspeed to slow=9fps(=1Mbit-stream at 640*480) for motion to not overload the wifi network	with 4 cameras


    if (!xTaskCreatePinnedToCore(&streamtask, "streamserver", 8192, NULL, tskIDLE_PRIORITY+5, &servertask, 1)) // extra streaming task on port 81
    {
        ESP_LOGE(TAG, "***Failed to create stream servertask task");
    }

    tcpserver(80); // server on port 80 to serve camera controls and stills

// we should never get here!
    xTimerDelete( tmr,0 );

}

void streamtask(void *param)
{
    tcpserver(81);
}

// main for the tcp webserver custom. This may be a task!
int tcpserver(int port)
{
    char request[400];
    int serverSocket, clientConn, ret,cnt=0;
    struct sockaddr_in IpAddress;  // this is an overlay for the struct sockaddr, that eases the portnumber entry.ie. overlays char sa_data[14] with WORD port, ULONG address
    IpAddress.sin_family = AF_INET;
    IpAddress.sin_port = htons(port); // the port to listen on   **************    this Port ***********************************
    IpAddress.sin_addr.s_addr = INADDR_ANY;//inet_addr("192.168.1.11"); INADDR_ANY, if you dont know it
    socklen_t socklen = sizeof(IpAddress);

    // open internet socket/endpoint for HTTP communication. return file handle or -1=error
    serverSocket = socket(
                       AF_INET,      // Domain: IPv4 Internet protocols
                       SOCK_STREAM,  // Communication-Type:  SOCK_STREAM=TCP; SOCK_DGRAM=UDP
                       IPPROTO_TCP            // was 0: Protocol: 0=IP,internet protocol, pseudo protocol number.TCP and UDP
                   );
    if (serverSocket < 0)
    {
        ESP_LOGE(TAG,"\nsocket failed");
        return -1;
    }


    // assign a specific internet address to the socket. return 0=OK, -1=error
    // normally the local loopback address is assigned. The Port is the one you opened on your router for the machines local LAN address.
    ret=bind(serverSocket, (struct sockaddr *) &IpAddress, socklen );
    if (ret)
    {
        ESP_LOGE(TAG,"\nbind failed");
        return -1;
    }
    // start listening on the socket. returns 0=OK, -1=error
    // The second parameter sets the queue_len for incoming requests.ie. MaxRequests.
    ret = listen(serverSocket, 5);
    if (ret)
    {
        ESP_LOGE(TAG,"\nlisten failed");
        return -1;
    }


    ESP_LOGI(TAG,"Server started on:%s:%u    running on CPUCore:%d", inet_ntoa(IpAddress.sin_addr),ntohs(IpAddress.sin_port),xPortGetCoreID() );

    // wait for connection. We only support 5 connection requests at a time. See listen() above
    // while there are connection requests in the input queue of the socket, process then.
    while(1)
    {
        // wait forever for next tcp connection request from the input queue.
        clientConn = accept(serverSocket, (struct sockaddr *) &IpAddress, &socklen); //this blocks !!
        //printf( "Client connect from: %s:%u\n", inet_ntoa(IpAddress.sin_addr),ntohs(IpAddress.sin_port) );

        // connection is established. loop until closed. So we only allow one connection at a time!
        while (1)
        {
            cnt++;
            // use also poll or select to monitor connection!
            ret = read(clientConn,request,sizeof(request)); //wait for new data on the connection. This blocks!!
            if (ret <= 0)
            {
                //printf("read failed!\n");
                break; // connection lost.  a 0 indicates an orderly disconnect by client; -1 some error occured.
            }
            request[ret]=0;
            //printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Got:\n %sR-EOT\n",request);

            // process response here.....
            ret=http_response(port, request, clientConn);
            //printf("End of Transaction %d <<<<<<<<<<<<<<<<<<<<<<<<\n",cnt);

            if (!ret) break; //close

        }

        //printf("Connection closed\n");
        close(clientConn); // close current tcp connection

    } // endwhile

    return 0;
}

/*
Request-Line = Method SPACE Request-URI SPACE HTTP-Version CRLF
we only support GET requests!
The request URI contains options on which item is requested!
HTTP-Version: always HTTP1.1.

entry: the complete request string
This routine builds and sends the response!
exit: 1= keep connection; 0=drop connection!
*/
const char *resp_index="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nContent-Encoding: gzip\r\n\r\n";
const char *resp_basic="HTTP/1.1 %s\r\n\r\n";
const char *resp_attach="HTTP/1.1 200 OK\r\nContent-Disposition: attachment; filename=\"frame.raw\"\r\nContent-Length: %d\r\n\r\n";
const char *resp_attach_bmp="HTTP/1.1 200 OK\r\nContent-Disposition: attachment; filename=\"frame.bmp\"\r\nContent-Length: %d\r\n\r\n";
const char *resp_capture="HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nContent-Disposition: inline; filename=capture.jpg\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_capture_bmp="HTTP/1.1 200 OK\r\nContent-Type: image/bmp\r\nContent-Length: %d\r\nContent-Disposition: inline; filename=capture.jpg\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_status="HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *resp_control="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nAccess-Control-Allow-Origin: *\r\n\r\n";

int http_response(int port, char *req, int connection)
{
    // the webpage source is included in the program! This will get the start/end address ..and the length
    extern const unsigned char index_ov7670_html_gz_start[] asm("_binary_index_ov7670_html_gz_start");
    extern const unsigned char index_ov7670_html_gz_end[] asm("_binary_index_ov7670_html_gz_end");
    int indexlength = index_ov7670_html_gz_end - index_ov7670_html_gz_start;
    uint8_t *pb;
    size_t len;
    char response[1024];

    char request[10], uri[100];
    int ret,keepalive=1;
    int more=0;
    int freeflag=0; // if convert, the jpg buffer must be freed!!
	int sendbmp=0;

    uint16_t regval;
    //printf("\n\nLength:%d\n",strlen(req));
    // check for GET: "GET / HTTP/1.1CRLF"
    ret=sscanf(req,"%s %s ",request,uri);
    //printf("req:%s uri:%s ret:%d\n",request,uri,ret);
    if (ret != 2) // some strange request
    {
        sprintf(response,resp_basic,"400 Bad Request");
        goto sendresponse;
    }

    if (strcmp(request,"GET")) // not a GET request
    {
        sprintf(response,resp_basic,"501 Not Implemented");
        goto sendresponse;
    }

    if (port == 80) // control port
    {
        //we are now at GET
        if (!strcmp(uri,"/")||!strcmp(uri,"/index.html")) // request for index.html
        {
            // send the webpage
            sprintf(response,resp_index,indexlength);
            pb=(uint8_t*)index_ov7670_html_gz_start;
            len=indexlength;
            goto sendmore;
        }


        // send status
        if (!strcmp(uri,"/status"))
        {
            if (!get_camstatus()) iobuf[0]=0;
            sprintf(response,resp_status,strlen(iobuf));
            strcat(response,iobuf);
            goto sendresponse;
        }


        // set control
        if (!strncmp(uri,"/control",8))
        {
            //printf("+++++++++++++control\n");
            ret = set_control(uri);
//        if (ret != 1) goto send404;  webpage freezes if 404 is returned, so dont do it
            sprintf(response,resp_control,0);
            goto sendresponse;
        }

        // Set/Get register value
        if ( !strncmp(uri,"/reg",4) ) // set a register
        {
            regval=set_register(uri);
            sprintf(response,resp_control,0);
            goto sendresponse;

        }

        if ( !strncmp(uri,"/greg",5) ) // get a register
        {
            regval=set_register(uri);
            sprintf(iobuf,"%u",regval);
            sprintf(response,resp_status,strlen(iobuf));
            strcat(response,iobuf);
            goto sendresponse;
        }

        if ( !strncmp(uri,"/getstatus",10) ) // getstatus from server
        {
            ret = get_status(uri);
            sprintf(response,resp_control,strlen(iobuf));
            strcat(response,iobuf);
            goto sendresponse;
        }


        // download raw image!! usually yuv422 like on ov7670, but jpg on ov7670.
        if (!strcmp(uri,"/download"))
        {
            if (!IsStreaming) //if currently streaming 0 Bytes will be downloaded!
            {

                if (!get_frame(0,&pb,&len)) len=0; // get raw frame
            }
            else len=0;
			
			if (UseBmp)
			{
				len+=BMPHDSIZE;
           sprintf(response,resp_attach_bmp,len);
		   sendbmp=1;
			}
			else
		    sprintf(response,resp_attach,len);
            goto sendmore;

        }

        // capture image!! only if not streaming, as the camera driver gets confused when calling get_frame in 2 tasks at the same time!
        if (!strncmp(uri,"/capture",8))
        {
            if (!IsStreaming)
            {
				ESP_LOGI(TAG,"Get Snapshot");

                if (Convert) freeflag=1; // if convert, the jpg buffer must be freed!!

                if (flashlight)
                {
                    gpio_set_level(4, 1); // turn led on
                    vTaskDelay(400/portTICK_PERIOD_MS);  // wait a little to get camera exposure settle to new light conditions

                }

                ret=get_frame(Convert,&pb,&len);
				

                gpio_set_level(4, 0); // turn led off
                if (!ret) len=0;
            }
            else len=0;
			
			if (UseBmp)
			{
				len+=BMPHDSIZE;
           sprintf(response,resp_capture_bmp,len);
		   sendbmp=1;
			}
			else
			{
            // printf("--pbuf:0x%08x len:%d\n",(uint32_t)pb,len);
            sprintf(response,resp_capture,len);
			}

            goto sendmore;


        }


    } // endif control port 80

// this if we are the streaming server!
    if (port == 81)
    {
        // http stream
        if (!strncmp(uri,"/stream",7))
            return(http_stream(connection,Convert));
    }

//default, nothing has catched: send 404 not found/supported----this upsets the client as it waits forever,blocks other controls ...maybe just send http ok??!!
// so we just send a dummy response. The html page does not support catching the error codes!
    ESP_LOGE(TAG,"Unknown GET request: %s",uri);
//    sprintf(response,resp_basic,"404 not found");
    sprintf(response,resp_control,0); // dummy OK

    goto sendresponse;

sendmore:
    more=1; // send data also
    keepalive=1; // keep connection
	
sendresponse:
    //printf(">>>>send response:\n%sT-EOT\n",response);
    if (more)
    {
        ret = MSG_MORE;
    }
    else ret = 0;

    // send the response text
    ret=send(connection, response, strlen(response),ret);// this blocks until data is sent. ret contains optional MSG_MORE flag to delay sending
    if (ret <= 0)
        return 0; // connection closed. broken connection
    if (ret != strlen(response)) ESP_LOGE(TAG,"send1, not all bytes sent:%d",ret);
    if (more)
    {
		if (sendbmp) // send bmp header first
		{
			send(connection, &BMPhead,BMPHDSIZE,MSG_MORE);
			sendbmp=0;
		}
        //printf("sending data...\n");
        ret = send(connection, pb, len, 0);// this blocks until data is sent

        if (freeflag) free(pb); // if convert, the jpg buffer must be freed!!

        esp_camera_fb_return(fb); // give back the used framebuffer to the camera driver to be used again
        fb=NULL; // mark free
        if (ret <= 0) // connection closed. broken connection
            return 0;
        if (ret != len) ESP_LOGE(TAG,"send2, not all bytes sent:%d",ret);

    }

// check if reset command was given:
    if (resetflag) 	esp_restart();  // we die from here

    return keepalive;	//in http1.1, always keep connection alive, unless someone hangs up. so always return 1.
}


const char *resp_stream="HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=ESP32CAM_ServerPush\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
const char *frame_header_jpg ="\r\n--ESP32CAM_ServerPush\r\nContent-Type:image/jpeg\r\nContent-Length:%d\r\n\r\n";
const char *frame_header_bmp ="\r\n--ESP32CAM_ServerPush\r\nContent-Type:image/bmp\r\nContent-Length:%d\r\n\r\n";
//const char *frame_header_jpg ="--ESP32CAM_ServerPush\r\n\r\n";
/* keep a streaming video until remote client hangs up
The content type multipart/x-mixed-replace was developed as part of a technology to emulate server push and streaming over HTTP.
This implements "The Multipart Content-Type" over HTTP Protocol using boundary-identifier.
This is not to be confused with chunked!!
The identifier can be any string you like;) must stay the same of corse.
MJEPG is working, of corse;)
RAW-YUV results in big data being transmitted, let see. test with firefox...no luck yet.
- convert = 0=format as of camera; 1= convert to ? jpg or bmp in get_frame
returns 0 = close connection
*/
int http_stream(int connection, int convert)
{
    uint8_t *pb;
    size_t len;
    char response[512];
	int ret;



    ESP_LOGI(TAG,"Stream Start....");


    ret=send(connection, resp_stream, strlen(resp_stream),0);
    if (ret <= 0) return 0; //client hanged up

    IsStreaming=1;

    while (1)
    {
        if (streamlight) gpio_set_level(4, 1); // turn led on
        else gpio_set_level(4, 0); // turn led off

        ret=get_frame(convert,&pb,&len);
        if (!ret) // error message is printed in driver if fails
        {
            // something went wrong in the camera/driver, just reset the thing trying to resolve it.
            ESP_LOGE(TAG,"Frame Capture failed....Restarting System now...............>>>>>>\n");
            fflush(stdout);
            esp_restart();
            break;
        }

		if (UseBmp)
		{
		//ESP_LOGI(TAG,"DataLength:%d",len);
        sprintf(response,frame_header_bmp,len+BMPHDSIZE); // + bmpheaderlength!!!
        ret=send(connection, response, strlen(response),MSG_MORE);
        ret=send(connection, &BMPhead, BMPHDSIZE,MSG_MORE); // send the bmp header before the picture data
		}
		else
		{
        sprintf(response,frame_header_jpg,len);
        ret=send(connection, response, strlen(response),MSG_MORE);
		}
        if (ret <= 0) //connection closed by client
        {
            if (convert) free(pb);
            break;
        }
        ret = send(connection, pb, len, 0);// send image . this blocks until data is sent
        if (convert) free(pb); // jpg conversion buffer
        esp_camera_fb_return(fb);
        fb=NULL;
        NetFrameCnt++; // calc FPS
        if (ret <= 0) break; //connection closed by client
        else if (ret != len) ESP_LOGE(TAG,"sendjpg, not all bytes sent:%d errno:%d",ret,errno);
    }

    IsStreaming=0;
    gpio_set_level(4, 0); // turn led off
    ESP_LOGI(TAG,"....Stream Stop");

    return 0; // hangup
}


/*
get a frame from the camera and optionally convert to jpg
uses global pointer to  fb_struct (camera framebuffer)
entry:
- convert flag, if 1, convert to jpg. This option shall only be 1, if camera produces non-jpg format like yuv422 (0v7670!)
- address of pointer to receive the resulting framebuffer address. if convert, its the newly allocated jpg-buffer and must be freed!!
- address of len variable receiving the length of framebuffer data.
exit:
1=OK, 0=capture or convert failed.
The buffer with data and the length is returned to caller using pointers!!
*/
int get_frame(int convert, uint8_t **buf, size_t *len)
{
    esp_camera_fb_return(fb); //release a possible last used framebuffer. we should work with at least 2 framebuffers!
	//ESP_LOGI(TAG,"geeting frame..\n");
    fb = esp_camera_fb_get(); // get a new framebuffer with current picture data
    if (!fb)
    {
        ESP_LOGE(TAG,"CamCapture failed");
        CapErrors++;
        return 0;
    }
    else
    {
        // save data from the framebuffer
        *buf=fb->buf;
        *len=fb->len;
    }
    if (convert)
    {
       // ESP_LOGI(TAG,"\nstart jpg convert(%u)...\n",*len);
        if (frame2jpg(fb, Quality, buf, len) != 1)
        {
            CapErrors++;
            //printf("jpg-convert failed\n");
            return 0;
        }
       //ESP_LOGI(TAG,"...convert done(%u)!\n",*len);

    }
		//ESP_LOGI(TAG,"...done\n");

    return 1;
}


/* process a register get/set command from client:
ov7670 has byte registers!
If you click a control button on the webpage, it will send the changed control value to us.
We need to forward it to the camera.
entry:
- uri-string containing the json formatted request: fe. URI: "/reg?reg=12296&mask=255&val=2" or "/greg?reg=12296&mask=255"
exit:
  the value returned or 0=OK. no error checking!!!
*/
static uint16_t set_register(char *uri)
{
    char *pfunction, *preg, *pvalue;
    int value=-1,reg,mask,setflag=0;
    sensor_t *s;
    // get json parameters from uri
    uri++; //skip leading /
    pfunction=strtok(uri, "?"); //returns "reg" or "greg"
    strtok(NULL, "=");//returns: "reg"
    preg=strtok(NULL, "&");//  returns: "12296"
    reg=atoi(preg);
    strtok(NULL, "="); // returns: "mask",
    preg=strtok(NULL, "&");//  returns: "255"
    mask=atoi(preg);

    if (!strcmp(pfunction,"reg"))
    {
        setflag=1; // we are setting a register
        strtok(NULL, "="); // returns: "val"
        pvalue=strtok(NULL, "="); // returns: "2". didnt find '=' but returns the last string
        value=atoi(pvalue);
    }
    ESP_LOGI(TAG, "Register: %s reg=0x%02x mask=0x%02x value:0x%02x", pfunction, reg, mask, value);

    s = esp_camera_sensor_get(); // get the cameras settings

    if (setflag)
    {
        if (s->set_reg) return(s->set_reg(s,reg,mask, value)); // set register with value. mask always 0xff
    }
    else
    {
        if (s->get_reg) return(s->get_reg(s,reg,mask)); // get register with value. mask always 0xff
    }
    ESP_LOGI(TAG, "register function not supported!");
    return 0;
}


/* process a set control command from client:
This usually is used to set some parameter in the camera, or to set some functionality on the server side.
entry:
- uri-string containing the json formatted request: fe. URI: /control?var=streamlight&val=0
exit:
- 1 = OK
- 0 = fail, control not found.
-1 = set function failed.
*/
static int set_control(char *uri)
{
    char *variable, *ps;
    int value,ret;
    sensor_t *s = esp_camera_sensor_get(); // get the cameras function list

    int  (*func)(sensor_t *sensor, int val)=NULL;
    // get json parameters from uri
    strtok(uri, "=&"); //goto first & or = .tell strtok to use string uri. returns: "/control?var"
    variable=strtok(NULL, "=&");// we are now at '='.  from last = find next = or & and put a /0 there. returns: "streamlight"
    strtok(NULL, "="); // returns: "val", skip it
    ps=strtok(NULL, "="); // returns: "0". didnt find '=' but returns the last string being the value
    value=atoi(ps);
    ESP_LOGI(TAG, "Control: %s = %d", variable, value);

//first check for internal commands for the server:
    ret=0;
    if (!strcmp(variable, "flashlight"))
    {
        flashlight=value;
        ret=1;
    }
    else if (!strcmp(variable, "streamlight"))
    {
        streamlight=value;
        ret=1;
    }
    else if (!strcmp(variable, "bmpmode"))
    {
        bmp_mode(value);
        ret=1;
    }
    else if (!strcmp(variable, "pixfmt"))
    {
        if (value)
		{
		s->set_pixformat(s,PIXFORMAT_YUV422); 
//		bmp_mode(0);
		}
		else
		{
		s->set_pixformat(s,PIXFORMAT_RGB565); 
		}
        ret=1;
    }
    else if (!strcmp(variable, "nightmode"))
    {
        night_mode(value);
        ret=1;
    }
    else if (!strcmp(variable, "esp32reset"))
    {
        resetflag = 1;
        ret=1;
    }
    else if (!strcmp(variable, "quality")) // adjust jpg quality in conversion routine
    {
       // scale 4 - 63 to 4 - 100. the higher the value the higher image quality and data transmitted
	   Quality = ((value - 4)*100)/(63 - 4); // convert range to percent
	   ESP_LOGI(TAG, "Set JPG-Quality to %d percent",Quality);
        ret=1;
    }

    else if (!strcmp(variable, "framesize"))
    {
        func=(void*)s->set_framesize;
		(*func)(s,value); // set the framesize
        nightmode=0;
		setbmp();
		ret = 1;
    }
    if (ret) return 1; //OK

    // its a camera setting command:
 
    if (!strcmp(variable, "brightness")) func = s->set_brightness;
    else if (!strcmp(variable, "contrast")) func = s->set_contrast;
    else if (!strcmp(variable, "saturation")) func = s->set_saturation;
    else if (!strcmp(variable, "special_effect")) func = s->set_special_effect;
    else if (!strcmp(variable, "awb")) func = s->set_whitebal;
    else if (!strcmp(variable, "wb_mode")) func = s->set_wb_mode;
    else if (!strcmp(variable, "awb_gain")) func = s->set_awb_gain;
    else if (!strcmp(variable, "aec")) func = s->set_exposure_ctrl;
    else if (!strcmp(variable, "aec_value")) func = s->set_aec_value;
    else if (!strcmp(variable, "ae_level")) func = s->set_ae_level;
    else if (!strcmp(variable, "aec2")) func = s->set_aec2;
    else if (!strcmp(variable, "agc")) func = s->set_gain_ctrl;
    else if (!strcmp(variable, "agc_gain")) func = s->set_agc_gain;
    else if (!strcmp(variable, "gainceiling")) func = (void*)s->set_gainceiling;
    else if (!strcmp(variable, "raw_gma")) func = s->set_raw_gma;
    else if (!strcmp(variable, "lenc")) func = s->set_lenc;
    else if (!strcmp(variable, "hmirror")) func = s->set_hmirror;
    else if (!strcmp(variable, "vflip")) func = s->set_vflip;
    else if (!strcmp(variable, "colorbar")) func = s->set_colorbar;
    else if (!strcmp(variable, "wpc")) func = s->set_wpc;
    else if (!strcmp(variable, "dcw")) func = s->set_dcw;
    else if (!strcmp(variable, "bpc")) func = s->set_bpc;
    //else the function is not supported: s->function=NULL
    if (func == NULL)
    {
        ESP_LOGE(TAG,"Control not supported");
        return 0; //setting not supported by camera
    }

//call the function:
    if ((*func)(s,value) != 0)
    {
        ESP_LOGE(TAG,"Camera Control failed");
        return -1; // set value failed
    }


    return 1; // OK
}

/* get a status info from the server:
This is a new feature to deliver data to the webpage from the server like current framerate
entry:
- uri-string containing the json formatted request: fe. URI: /getstatus?var=framerate
The var identifiys the parameter requested, here its framerate.
exit:
returns the requested  data in text form. it uses global iobuf to return the response text
- 1 = OK
- 0 = fail, '-1' is returned as text to webpage

*/
static int get_status(char *uri)
{
    char *variable;
    // get json parameters from uri
    strtok(uri, "=&"); //goto first & or = .tell strtok to use string uri. returns: "/getstatus?var"
    variable=strtok(NULL, "=&");// we are now at '='.  from last = find next = or & and put a /0 there. returns: "framerate"

    ESP_LOGI(TAG, "getstatus: %s", variable);

//check 'variable' for the request parameter
    if (!strcmp(variable, "framerate"))
    {
        sprintf(iobuf,"NetFPS:%d HwFPS:%d I2sFPS:%d CamErrors:%d",NetFPS,HwFPS,I2sFPS,CapErrors);
        return 1; //OK, answer in iobuf
    }

    sprintf(iobuf,"%d",-1);
    return 0; // no parameter-name match
}



/*
get current camera settings status. To be used to init the webpage after retrieval.
exit:
returns the requested  data in text form. it uses global iobuf to return the response text
- 1 = OK
- 0 = fail
*/
static int get_camstatus(void)
{
    sensor_t *s = esp_camera_sensor_get(); // get the status of camera controls from camera
    if (s == NULL) return 0;
    char *p = iobuf;
    // assemlbe them into a string
    *p++ = '{';

    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
    //internal server maintained settings:
    p += sprintf(p, ",\"nightmode\":%d", nightmode);
    p += sprintf(p, ",\"bmpmode\":%d", UseBmp);
    p += sprintf(p, ",\"flashlight\":%d", flashlight);
    p += sprintf(p, ",\"streamlight\":%d", streamlight);

    *p++ = '}';
    *p++ = 0;
    return 1;
}

/*set the streaming mode to jpg or bmp if sensor delivers RGB565!
1=bmp
0=jpg
*/
void bmp_mode(int bmp)
{

    sensor_t *s = esp_camera_sensor_get(); // get the cameras function list
	if (s->pixformat == PIXFORMAT_RGB565)
	{
		if (bmp)
		{
		UseBmp=1;
		Convert=0;
		}
		else
		{
		UseBmp=0;
		Convert=1;
		}
	}
	else
	{
		UseBmp=0;
		Convert=1;
	}

	ESP_LOGI(TAG,"StreamBMP:%d StreamJPG:%d",UseBmp,Convert);

}

/* turn nightmode on/off
The advantage of nightmode is, that exposuretime is much longer, so you get less noise compared to
just increasing the gain, but at lower framerates.

example: 640*480 at normal constant framerate of 25fps:

if on: the framerate varys from 3 fps(dark) to 25fps(light).
	   here longer exposure times are used and exposuretime can be longer than 1 frame ie. 7frames long .
	   thus the framerate is reduced if longer exposuretimes are needed.
if off: framerate is returned to normal constant state of fe. 25 fps, also in dark, so less sensitive.
        exposuretime can only be max 1 frame then.
		NOTE: AEC and AGC must be ON for nightmode to work as its the exposure/light-control engine.
		      (AEC and AGC are working closely together!)
		      Use AE-Level to adjust to best brightness.
			  The clock is changed to full speed after nightmode usage!
OV7670!!!			  
*/
void night_mode(int on)
{
    sensor_t *s = esp_camera_sensor_get(); // get the cameras function list

	uint8_t val;
	val = s->get_reg(s,0x3b,0xff);
    if (on) //turn on
    {
        val|=0xe0;
        nightmode=1;
    }
    else //turn off, is abit complicated
    {
        val&=0x1f;
        nightmode=0;
    }
	s->set_reg(s,0x3b,0xff, val);
}

// 1 sec peridic timer
void timerCallBack( TimerHandle_t xTimer )
{
    NetFPS=NetFrameCnt;
    NetFrameCnt=0;
    HwFPS=HwFrameCnt;
    HwFrameCnt=0;
	I2sFPS=I2sFrameCnt;
	I2sFrameCnt=0;

}


/*  BMP:
// this is an example of putting raw rgb565 data from a camera-sensor into a bmp file or image to stream via http.

// camera horizontal resolution must be dividable by 4!!,so we dont need padding and can just take the raw framedata.
// the beginnig of the image-data must start at divby4 address, so we need to make the header 68 bytes long, by adding 2 dummybytes at the end.
struct bmphead
{
	// BMP Header [14], always 14 Bytes long. 
	uint16_t id; //0:=0x424d; // BM    ByteOffset is given by 0:, below also.
	uint32_t filesize; //2: set this +++ picsize in bytes +68 for header.
	uint32_t reserved; //6: = 0;
	uint32_t headeroffset; //10: = 68-1; // start of picture data, offset from headerstart. seems count is: n-1, .(had bad colors otherwise!)

	// DIB Header [40]=fixed. info about pic-dimensions and colordepth
	uint32_t headersize; //14: = 40; // DIB Header  size in bytes, thats fixed to 40
	uint32_t width; //18: picture horizontal pixelcount set this +++
	uint32_t height; //22: picture vertical lines       set this +++
	uint16_t planes; //26: = 1; // no of planes
	uint16_t bitsperpixel; //28: = 16;
	uint32_t compression; //30: = 3; //3 bit fields
	uint32_t imagesize; //34: size of image data in bytes set this +++
	uint32_t pixpermeterx; //38: = 0; //not defined
	uint32_t pixpermetery; //42: = 0; //not defined
	uint32_t colorused; //46: = 0; // number of used colors = undefined
	uint32_t colorall; //50: = 0; // number of important colors = undefined
	// maskTable[12] contains bitmasks to exstract the rgb components from the 16bit word of rgb565 data.
	uint32_t rmask; //54: = 0xf800; //WORD: red color information mask; (pixel & red_mask) >> 11; BYTE red   = red_value << 3;
	uint32_t gmask; //58: = 0x07e0; // green; BYTE green_value = (pixel & green_mask) >> 5; BYTE green = green_value << 2;
	uint32_t bmask; //62: = 0x001f; // blue; BYTE blue_value = (pixel & blue_mask); BYTE blue  = blue_value << 3;
	uint16_t dummy; // to adjust to being dividable by 4. ie. uint32 size. but maybe work without the dummy?!.
// 68 Bytes!	
//picdata from here......

//            client.write(bmpHeader, BMP::headerSize);
//            client.write(camera->frame, camera->xres * camera->yres * 2);
	
};


make bmp header for given resolution
- *pbuf = pointer to a buffer of min size BMPHDSIZE (68)
- width = picture width (dividable by 4!)
- height = picture height/rows (any count)
- bpp = bytes per pixel (2 for rgb565/yuv422)
*/
void makebmpheader(uint8_t *pbuf, uint16_t width, uint16_t height, uint16_t bpp)
{
	int i,headersize = BMPHDSIZE;
	uint32_t l;
	// precelar buffer
	for (i=0;i<headersize;i++) *(pbuf+i)=0;
	
	// fill in the numbers
	*pbuf=0x42; // B
	*(pbuf+1)=0x4d; // M
	
	l=(width*height*bpp)+headersize; // bmp-filesize
	*(pbuf+2)=l;  // we have to convert the byte-order to the bmp standard!(little-endian) thats why we do it like this.
	*(pbuf+3)=l>>8;
	*(pbuf+4)=l>>16;
	*(pbuf+5)=l>>24;
	
	*(pbuf+10)=headersize-1; // somehow colors are only correct if headersize. headersize-1 give correct colors??!! try and error;).maybe n-1 counting
	*(pbuf+14)=40; //dib-hd-size
	
	*(pbuf+18)=width; 
	*(pbuf+19)=width>>8; 
	
	*(pbuf+22)=height; 
	*(pbuf+23)=height>>8; 
	
	*(pbuf+26)=1; //planes
	*(pbuf+28)=16; //bitsperpixel
	*(pbuf+30)=3; //compression, 3 rgb-bit fields defined in the masks below
	
	l=width*height*bpp; // imagesize of raw camera image appended after the header.
	*(pbuf+34)=l;
	*(pbuf+35)=l>>8;
	*(pbuf+36)=l>>16;
	*(pbuf+37)=l>>24;
	
	l=0xf800; // red color mask. look at rgb565 description! it matches the definition of the camera rgb bitfields.
	*(pbuf+54)=l;
	*(pbuf+55)=l>>8;
	
	l=0x07e0; // green color mask
	*(pbuf+58)=l;
	*(pbuf+59)=l>>8;
	
	l=0x001f; // blue color mask
	*(pbuf+62)=l;
	*(pbuf+63)=l>>8;
	// the rest is 0.
	
}

/* set the BMP header asto current framesize
exit:
1 = ok, BMPhead struct inited to current framesize
0 = fail, no rgb565 detected, so its yuv or something else! (on the ov7670)
*/
int setbmp(void)
{

	uint16_t width,height, bpp, colorfmt, framesize;
    sensor_t *s;
    s = esp_camera_sensor_get(); // get the cameras settings from camera.c driver with current settings
	colorfmt = s->pixformat;
	framesize = s->status.framesize;
	width = resolution[s->status.framesize].width;
	height = resolution[s->status.framesize].height;
	bpp = 2; //bytes per pixel
	makebmpheader(BMPhead, width, height, bpp);
	ESP_LOGI(TAG,"BMP Settings: w:%u h:%u framesize:%u colfmt:%u BmpMode:%d",width,height,framesize,colorfmt,UseBmp);

	return 0;

}

	