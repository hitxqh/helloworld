// MavlinkTest.cpp : 定义控制台应用程序的入口点。
//

//#include "stdafx.h"
//
//#include "minimal\mavlink.h"
//int _tmain(int argc, _TCHAR* argv[])
//{
//	
//	mavlink_heartbeat_t t;
//	mavlink_message_t message;
//	uint8_t buf[1024];
//	mavlink_msg_heartbeat_pack(1, 200, &message, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_AUTO_ENABLED, 0, MAV_STATE_ACTIVE);
//
//        // Copy the message to send buffer
//        uint16_t len = mavlink_msg_to_send_buffer(buf, &message);
//	mavlink_msg_heartbeat_decode(&message, &t);
//	return 0;
//}

#include <stdio.h>
#include <string.h>
#include <math.h>

typedef unsigned int uint32_t;
typedef int int32_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned char uint8_t;
typedef char int8_t;
#define BUFFER_SIZE     10

#define BUFFER_SIZE_INT     4
#define IMAGE_WIDTH  64
#define TILE_SIZE	8               						// x & y tile size
#define NUM_BLOCKS	5 // x & y number of tiles to check

#define sign(x) (( x > 0 ) - ( x < 0 ))
#define SEARCH_SIZE 4
#define FRAME_SIZE 64
#define THRESHOLD 30
#define PARAM_BOTTOM_FLOW_VALUE_THRESHOLD 5000
#define PARAM_GYRO_COMPENSATION_THRESHOLD 0.01
#define GET_TIME_BETWEEN_IMAGES 2
unsigned char GetCharByPos(int a, char cIndex)
{
	char buffer[BUFFER_SIZE_INT+1] = {0};
	memcpy(buffer, &a, sizeof(int));
	return buffer[cIndex];
}

int usad8(int a, int b)
{
	int sum = 0;
	for(int i = 0; i < 4; i++){
		sum += abs(GetCharByPos(a,i)-GetCharByPos(b,i));
	}

	return sum;
}

int usada8(int a, int b, int c)
{
	return usad8(a,b)+c;
}

int uhadd8(int a, int b)
{
	char buffer[BUFFER_SIZE_INT+1] = {0};
	for(int i = 0; i < 4; i++){
		buffer[i] = (GetCharByPos(a,i)+GetCharByPos(b,i))/2;
	}

	int iRet;
	memcpy(&iRet, buffer, sizeof(int));
	return iRet;	
}














/**
* @brief Compute the average pixel gradient of all horizontal and vertical steps
*
* TODO compute_diff is not appropriate for low-light mode images
*
* @param image ...
* @param offX x coordinate of upper left corner of 8x8 pattern in image
* @param offY y coordinate of upper left corner of 8x8 pattern in image
*/


static inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off = (offY + 2) * row_size + (offX + 2); // we calc only the 4x4 pattern
	uint32_t acc;

	/* calc row diff */
	acc = usad8 (*((uint32_t*) &image[off + 0 + 0 * row_size]), *((uint32_t*) &image[off + 0 + 1 * row_size]));
	acc = usada8(*((uint32_t*) &image[off + 0 + 1 * row_size]), *((uint32_t*) &image[off + 0 + 2 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image[off + 0 + 2 * row_size]), *((uint32_t*) &image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 1 + 0 * row_size] << 16 | image[off + 2 + 0 * row_size] << 8 | image[off + 3 + 0 * row_size];
	uint32_t col2 = (image[off + 0 + 1 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 2 + 1 * row_size] << 8 | image[off + 3 + 1 * row_size];
	uint32_t col3 = (image[off + 0 + 2 * row_size] << 24) | image[off + 1 + 2 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 3 + 2 * row_size];
	uint32_t col4 = (image[off + 0 + 3 * row_size] << 24) | image[off + 1 + 3 * row_size] << 16 | image[off + 2 + 3 * row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calc column diff */
	acc = usada8(col1, col2, acc);
	acc = usada8(col2, col3, acc);
	acc = usada8(col3, col4, acc);

	return acc;

}

/**
* @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
*
* @param image1 ...
* @param image2 ...
* @param off1X x coordinate of upper left corner of pattern in image1
* @param off1Y y coordinate of upper left corner of pattern in image1
* @param off2X x coordinate of upper left corner of pattern in image2
* @param off2Y y coordinate of upper left corner of pattern in image2
* @param acc array to store SAD distances for shift in every direction
*/
static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	* calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	* every iteration is one line of the 8x8 field.
	*
	*  + - + - + - + - + - + - + - + - +
	*  |   |   |   |   |   |   |   |   |
	*  + - + - + - + - + - + - + - + - +
	*
	*
	*/

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		* first column of 4 pixels:
		*
		*  + - + - + - + - + - + - + - + - +
		*  | x | x | x | x |   |   |   |   |
		*  + - + - + - + - + - + - + - + - +
		*
		* the 8 s values are from following positions for each pixel (X):
		*  + - + - + - +
		*  +   5   7   +
		*  + - + 6 + - +
		*  +   4 X 0   +
		*  + - + 2 + - +
		*  +   3   1   +
		*  + - + - + - +
		*
		*  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		*
		*/

		/* compute average of two pixel values */
		s0 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+0) * row_size])));
		s1 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+1) * row_size])));
		s2 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i+1) * row_size])));
		s3 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+1) * row_size])));
		s4 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+0) * row_size])));
		s5 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i-1) * row_size])));
		s6 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i-1) * row_size])));
		s7 = (uhadd8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i-1) * row_size])));

		/* these 4 t values are from the corners around the center pixel */
		t1 = (uhadd8(s0, s1));
		t3 = (uhadd8(s3, s4));
		t5 = (uhadd8(s4, s5));
		t7 = (uhadd8(s7, s0));

		/*
		* finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		*  + - + - + - +
		*  |   |   |   |
		*  + - 5 6 7 - +
		*  |   4 X 0   |
		*  + - 3 2 1 - +
		*  |   |   |   |
		*  + - + - + - +
		*/

		/* fill accumulation vector */
		acc[0] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		acc[1] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		acc[2] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		acc[3] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		acc[4] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		acc[5] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		acc[6] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		acc[7] = usada8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		/*
		* same for second column of 4 pixels:
		*
		*  + - + - + - + - + - + - + - + - +
		*  |   |   |   |   | x | x | x | x |
		*  + - + - + - + - + - + - + - + - +
		*
		*/

		s0 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+0) * row_size])));
		s1 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+1) * row_size])));
		s2 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i+1) * row_size])));
		s3 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+1) * row_size])));
		s4 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+0) * row_size])));
		s5 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i-1) * row_size])));
		s6 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i-1) * row_size])));
		s7 = (uhadd8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i-1) * row_size])));

		t1 = (uhadd8(s0, s1));
		t3 = (uhadd8(s3, s4));
		t5 = (uhadd8(s4, s5));
		t7 = (uhadd8(s7, s0));

		acc[0] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		acc[1] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		acc[2] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		acc[3] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		acc[4] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		acc[5] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		acc[6] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		acc[7] = usada8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
	}

	return 0;
}

/**
* @brief Compute SAD of two 8x8 pixel windows.
*
* @param image1 ...
* @param image2 ...
* @param off1X x coordinate of upper left corner of pattern in image1
* @param off1Y y coordinate of upper left corner of pattern in image1
* @param off2X x coordinate of upper left corner of pattern in image2
* @param off2Y y coordinate of upper left corner of pattern in image2
*/
static inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	acc = usad8 (*((uint32_t*) &image1[off1 + 0 + 0 * row_size]), *((uint32_t*) &image2[off2 + 0 + 0 * row_size]));
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 0 * row_size]), *((uint32_t*) &image2[off2 + 4 + 0 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 1 * row_size]), *((uint32_t*) &image2[off2 + 0 + 1 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 1 * row_size]), *((uint32_t*) &image2[off2 + 4 + 1 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 2 * row_size]), *((uint32_t*) &image2[off2 + 0 + 2 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 2 * row_size]), *((uint32_t*) &image2[off2 + 4 + 2 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 3 * row_size]), *((uint32_t*) &image2[off2 + 0 + 3 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 3 * row_size]), *((uint32_t*) &image2[off2 + 4 + 3 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 4 * row_size]), *((uint32_t*) &image2[off2 + 0 + 4 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 4 * row_size]), *((uint32_t*) &image2[off2 + 4 + 4 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 5 * row_size]), *((uint32_t*) &image2[off2 + 0 + 5 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 5 * row_size]), *((uint32_t*) &image2[off2 + 4 + 5 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 6 * row_size]), *((uint32_t*) &image2[off2 + 0 + 6 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 6 * row_size]), *((uint32_t*) &image2[off2 + 4 + 6 * row_size]), acc);

	acc = usada8(*((uint32_t*) &image1[off1 + 0 + 7 * row_size]), *((uint32_t*) &image2[off2 + 0 + 7 * row_size]), acc);
	acc = usada8(*((uint32_t*) &image1[off1 + 4 + 7 * row_size]), *((uint32_t*) &image2[off2 + 4 + 7 * row_size]), acc);

	return acc;
}

/**
* @brief Computes pixel flow from image1 to image2
*
* Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
* and calculates the average offset of all.
*
* @param image1 previous image buffer
* @param image2 current image buffer (new)
* @param x_rate gyro x rate
* @param y_rate gyro y rate
* @param z_rate gyro z rate
*
* @return quality of flow calculation
*/
uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y) {

	/* constants */
	const int16_t winmin = -SEARCH_SIZE;//-4
	const int16_t winmax = SEARCH_SIZE;//4
	const uint16_t hist_size = 2*(winmax-winmin+1)+1;

	/* variables */
	uint16_t pixLo = SEARCH_SIZE + 1;//5
	uint16_t pixHi = FRAME_SIZE - (SEARCH_SIZE + 1) - TILE_SIZE;//64-5-8=51
	uint16_t pixStep = (pixHi - pixLo) / NUM_BLOCKS + 1;//(51-5)/5+1=10
	uint16_t i, j;
	uint32_t acc[8]; // subpixels
	uint16_t histx[hist_size]; // counter for x shift
	uint16_t histy[hist_size]; // counter for y shift
	int8_t  dirsx[64]; // shift directions in x
	int8_t  dirsy[64]; // shift directions in y
	uint8_t  subdirs[64]; // shift directions of best subpixels
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;

	/* initialize with 0 */
	for (j = 0; j < hist_size; j++) { histx[j] = 0; histy[j] = 0; }

	/* iterate over all patterns
	*/
	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{
			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, (uint16_t) IMAGE_WIDTH);
			if (diff < THRESHOLD)//30
			{
				continue;
			}

			uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;

			uint8_t *base1 = image1 + j * (uint16_t) IMAGE_WIDTH + i;

			for (jj = winmin; jj <= winmax; jj++)
			{
				uint8_t *base2 = image2 + (j+jj) * (uint16_t) IMAGE_WIDTH + i;

				for (ii = winmin; ii <= winmax; ii++)
				{
					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, IMAGE_WIDTH);
					//uint32_t temp_dist = ABSDIFF(base1, base2 + ii);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			//dist is the biggest 
			if (dist < PARAM_BOTTOM_FLOW_VALUE_THRESHOLD)
			{
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, (uint16_t) IMAGE_WIDTH);
				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction
				for(uint8_t k = 0; k < 8; k++)
				{
					if (acc[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}
				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				/* feed histogram filter*/
				uint8_t hist_index_x = 2*sumx + (winmax-winmin+1);
				if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) hist_index_x += 1;
				if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) hist_index_x += -1;
				uint8_t hist_index_y = 2*sumy + (winmax-winmin+1);
				if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) hist_index_y += 1;
				if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) hist_index_y += -1;

				histx[hist_index_x]++;
				histy[hist_index_y]++;

			}
		}
	}

	/* create flow image if needed (image1 is not needed anymore)
	* -> can be used for debugging purpose
	*/
	//	if (global_data.param[PARAM_USB_SEND_VIDEO] )//&& global_data.param[PARAM_VIDEO_USB_MODE] == FLOW_VIDEO)
	//	{
	//
	//		for (j = pixLo; j < pixHi; j += pixStep)
	//		{
	//			for (i = pixLo; i < pixHi; i += pixStep)
	//			{
	//
	//				uint32_t diff = compute_diff(image1, i, j, (uint16_t) IMAGE_WIDTH);
	//				if (diff > THRESHOLD)
	//				{
	//					image1[j * ((uint16_t) IMAGE_WIDTH) + i] = 255;
	//				}
	//
	//			}
	//		}
	//	}

	/* evaluate flow calculation */
	if (meancount > 10)
	{
		meanflowx /= meancount;
		meanflowy /= meancount;

		int16_t maxpositionx = 0;
		int16_t maxpositiony = 0;
		uint16_t maxvaluex = 0;
		uint16_t maxvaluey = 0;

		/* position of maximal histogram peek */
		for (j = 0; j < hist_size; j++)
		{
			if (histx[j] > maxvaluex)
			{
				maxvaluex = histx[j];
				maxpositionx = j;
			}
			if (histy[j] > maxvaluey)
			{
				maxvaluey = histy[j];
				maxpositiony = j;
			}
		}

		/* check if there is a peak value in histogram */
		if (1) //(histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)
		{
			if (0)//0
			{

				/* use histogram filter peek value */
				uint16_t hist_x_min = maxpositionx;
				uint16_t hist_x_max = maxpositionx;
				uint16_t hist_y_min = maxpositiony;
				uint16_t hist_y_max = maxpositiony;

				/* x direction */
				if (maxpositionx > 1 && maxpositionx < hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 2;
				}
				else if (maxpositionx == 0)
				{
					hist_x_min = maxpositionx;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-1)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx;
				}
				else if (maxpositionx == 1)
				{
					hist_x_min = maxpositionx - 1;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 1;
				}

				/* y direction */
				if (maxpositiony > 1 && maxpositiony < hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == 0)
				{
					hist_y_min = maxpositiony;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-1)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony;
				}
				else if (maxpositiony == 1)
				{
					hist_y_min = maxpositiony - 1;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 1;
				}

				float hist_x_value = 0.0f;
				float hist_x_weight = 0.0f;

				float hist_y_value = 0.0f;
				float hist_y_weight = 0.0f;

				for (uint8_t i = hist_x_min; i < hist_x_max+1; i++)
				{
					hist_x_value += (float) (i*histx[i]);
					hist_x_weight += (float) histx[i];
				}

				for (uint8_t i = hist_y_min; i<hist_y_max+1; i++)
				{
					hist_y_value += (float) (i*histy[i]);
					hist_y_weight += (float) histy[i];
				}

				histflowx = (hist_x_value/hist_x_weight - (winmax-winmin+1)) / 2.0f ;
				histflowy = (hist_y_value/hist_y_weight - (winmax-winmin+1)) / 2.0f;

			}
			else
			{

				/* use average of accepted flow values */
				uint32_t meancount_x = 0;
				uint32_t meancount_y = 0;

				for (uint8_t i = 0; i < meancount; i++)
				{
					float subdirx = 0.0f;
					if (subdirs[i] == 0 || subdirs[i] == 1 || subdirs[i] == 7) subdirx = 0.5f;

					if (subdirs[i] == 3 || subdirs[i] == 4 || subdirs[i] == 5) subdirx = -0.5f;
					histflowx += (float)dirsx[i] + subdirx;
					meancount_x++;

					float subdiry = 0.0f;
					if (subdirs[i] == 5 || subdirs[i] == 6 || subdirs[i] == 7) subdiry = 0.5f;
					if (subdirs[i] == 1 || subdirs[i] == 2 || subdirs[i] == 3) subdiry = -0.5f;
					histflowy += (float)dirsy[i] + subdiry;
					meancount_y++;
				}

				histflowx /= meancount_x;
				histflowy /= meancount_y;

			}

			/* compensate rotation */
			/* calculate focal_length in pixel */
			//global_data.param[PARAM_FOCAL_LENGTH_MM]=16.0
			//focal_length_px = 2/3*1000
			const float focal_length_px = (16.0) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled

			/*
			* gyro compensation
			* the compensated value is clamped to
			* the maximum measurable flow value (param BFLOW_MAX_PIX) +0.5
			* (sub pixel flow can add half pixel to the value)
			*
			* -y_rate gives x flow
			* x_rates gives y_flow
			*/
			if (false)
			{
				if(fabsf(y_rate) > PARAM_GYRO_COMPENSATION_THRESHOLD)
				{
					/* calc pixel of gyro */
					float y_rate_pixel = y_rate * (GET_TIME_BETWEEN_IMAGES / 1000.0f) * focal_length_px;
					float comp_x = histflowx + y_rate_pixel;

					/* clamp value to maximum search window size plus half pixel from subpixel search */
					if (comp_x < (-SEARCH_SIZE - 0.5f))
						*pixel_flow_x = (-SEARCH_SIZE - 0.5f);
					else if (comp_x > (SEARCH_SIZE + 0.5f))
						*pixel_flow_x = (SEARCH_SIZE + 0.5f);
					else
						*pixel_flow_x = comp_x;
				}
				else
				{
					*pixel_flow_x = histflowx;
				}

				if(fabsf(x_rate) > PARAM_GYRO_COMPENSATION_THRESHOLD)
				{
					/* calc pixel of gyro */
					float x_rate_pixel = x_rate * (GET_TIME_BETWEEN_IMAGES / 1000.0f) * focal_length_px;
					float comp_y = histflowy - x_rate_pixel;

					/* clamp value to maximum search window size plus/minus half pixel from subpixel search */
					if (comp_y < (-SEARCH_SIZE - 0.5f))
						*pixel_flow_y = (-SEARCH_SIZE - 0.5f);
					else if (comp_y > (SEARCH_SIZE + 0.5f))
						*pixel_flow_y = (SEARCH_SIZE + 0.5f);
					else
						*pixel_flow_y = comp_y;
				}
				else
				{
					*pixel_flow_y = histflowy;
				}

				/* alternative compensation */
				//				/* compensate y rotation */
				//				*pixel_flow_x = histflowx + y_rate_pixel;
				//
				//				/* compensate x rotation */
				//				*pixel_flow_y = histflowy - x_rate_pixel;

			} else
			{
				/* without gyro compensation */
				*pixel_flow_x = histflowx;
				*pixel_flow_y = histflowy;
			}

		}
		else
		{
			*pixel_flow_x = 0.0f;
			*pixel_flow_y = 0.0f;
			return 0;
		}
	}
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

	/* calc quality */
	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));

	return qual;
}


#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
using namespace std;
using namespace cv;

int main()
{
	VideoCapture capture;
	Mat frame;
	Mat lastframe;

	//-- 2. Read the video stream
	capture.open( 0 );
	capture.set(CV_CAP_PROP_FRAME_WIDTH,64);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,64);
	if ( ! capture.isOpened() ) { printf("--(!)Error opening video capture\n"); return -1; }

	while ( capture.read(frame) )
	{
		if( frame.empty() )
		{
			printf(" --(!) No captured frame -- Break!");
			break;
		}



		if( !lastframe.empty() )
		{
			float x, y;
			compute_flow(frame.data, lastframe.data, 1,1,1,&x, &y);
			int i = 0;

		}


		lastframe = frame;

		imshow( "", frame );


		int c = waitKey(10);
		if( (char)c == 27 ) { break; } // escape
	}
	return 0;

}