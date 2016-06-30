/*
 * image.cpp
 *
 *  Created on: 01/03/2012
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot/data.hpp>
#include "image.hpp"
#include "jpeg.hpp"
#include <crosbot/utils.hpp>

#include <netinet/in.h>
#include <sys/stat.h>
#include <sys/time.h>
//#include <utime.h>
#include <png.h>

namespace crosbot {

namespace image {

#define CASROS_JPEG_COMPRESSION_QUALITY		80

#define CLAMP_255(C)	((C) < 0?0:((C) > 255?255:(C)))

#ifndef __UINT8_MAX__
#define __UINT8_MAX__		255
#endif

#ifndef __UINT16_MAX__
#define __UINT16_MAX__		65535
#endif

#ifndef __UINT32_MAX__
#define __UINT32_MAX__		4294967295U
#endif


/*
 * These conversion formulas are taken from Wikepedia (http://en.wikipedia.org/wiki/YUV) unless otherwise stated.
 */
#ifdef COLOUR_FLOAT_CONVERSIONS

inline void casros_convert_rgb_yuv(int R, int G, int B, int *Y, int *U, int *V) {
	*Y = (int)(0.299*R + 0.587*G + 0.114*B);
	*U = (int)((0.436*B - 0.14713*R - 0.28886*G) / 0.872) + 128;
	*V = (int)((0.615*R - 0.51499*G - 0.10001*B) / 1.230) + 128;
	*Y = CLAMP_255(*Y);
	*U = CLAMP_255(*U);
	*V = CLAMP_255(*V);
}

inline void casros_convert_yuv_rgb(int Y, int U, int V, int *R, int *G, int *B) {
	float UF = (U - 128) * 0.872;
	float VF = (V - 128) * 1.230;
	*R = (int)(Y + 1.13983*VF);
	*G = (int)(Y - 0.39465*UF - 0.58060*VF);
	*B = (int)(Y + 2.03211*UF);
	*R = CLAMP_255(*R);
	*G = CLAMP_255(*G);
	*B = CLAMP_255(*B);
}

#else // COLOUR_FLOAT_CONVERSIONS

inline void casros_convert_rgb_yuv(int R, int G, int B, int *Y, int *U, int *V) {
	*Y = ((66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
	*U = ((-38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
	*V = ((112 * R -  94 * G -  18 * B + 128) >> 8) + 128;
	*Y = CLAMP_255(*Y);
	*U = CLAMP_255(*U);
	*V = CLAMP_255(*V);
}

//inline void casimage_convert_yuv_rgb(int Y, int U, int V, int *R, int *G, int *B) {
//	int C = Y - 16;
//	int D = U - 128;
//	int E = V - 128;
//	*R = (298 * C + 409 * E + 128) >> 8;
//	*G = (298 * C - 100 * D - 208 * E + 128) >> 8;
//	*B = (298 * C + 516 * D + 128) >> 8;
//	*R = CLAMP_255(*R);
//	*G = CLAMP_255(*G);
//	*B = CLAMP_255(*B);
//}

//inline void casimage_convert_yuv_rgb(int Y, int U, int V, int *R, int *G, int *B) {
//	U -= 128; V -= 128;
//	*R = Y + V + (V >> 2) + (V >> 3) + (V >> 5);
//	*G = Y - ((U >> 2) + (U >> 4) + (U >> 5)) - ((V >> 1) + (V >> 3) + (V >> 4) + (V >> 5));
//	*B = Y + U + (U >> 1) + (U >> 2) + (U >> 6);
//	*R = CLAMP_255(*R);
//	*G = CLAMP_255(*G);
//	*B = CLAMP_255(*B);
//}

/**
 * This formula is copied from Raymond Sheh's VideoShim code.
 */
inline void casros_convert_yuv_rgb(int Y, int U, int V, int *R, int *G, int *B) {
	Y -= 16; V -= 128; U -= 128;
	*R = (9535*Y+13074*V) >> 13;
	*G = (9535*Y-6660*V-3203*U) >> 13;
	*B = (9535*Y+16531*U) >> 13;
	*R = CLAMP_255(*R);
	*G = CLAMP_255(*G);
	*B = CLAMP_255(*B);
}

#endif // COLOUR_FLOAT_CONVERSIONS

bool upSample8To16(Image& dest, const Image& src) {
	uint64_t numSamples = dest.height * dest.width *
			Image::numberOfChannels(dest.encoding);

	if (dest.dataLength != 2*numSamples || dest.data == NULL ||
			src.dataLength != numSamples || src.data == NULL) {
		return false;
	}

	uint8_t* srcData = (uint8_t *)src.data;
	uint16_t* destData = (uint16_t *)dest.data;
	uint32_t tmp;
	for (uint64_t i = 0; i < numSamples; i++, srcData++, destData++) {
		tmp = *srcData;
		tmp = (tmp << 8) | tmp;
		*destData = tmp;
	}

	return true;
}

bool upSample8To32(Image& dest, const Image& src) {
	uint64_t numSamples = dest.height * dest.width *
			Image::numberOfChannels(dest.encoding);

	if (dest.dataLength != 4*numSamples || dest.data == NULL ||
			src.dataLength != numSamples || src.data == NULL) {
		return false;
	}

	uint8_t* srcData = (uint8_t *)src.data;
	uint32_t* destData = (uint32_t *)dest.data;
	uint32_t tmp;
	for (uint64_t i = 0; i < numSamples; i++, srcData++, destData++) {
		tmp = *srcData;
		tmp = (tmp << 24) | (tmp << 16) | (tmp << 8) | tmp;
		*destData = tmp;
	}

	return true;
}

bool upSample16To32(Image& dest, const Image& src) {
	uint64_t numSamples = dest.height * dest.width *
			Image::numberOfChannels(dest.encoding);

	if (dest.dataLength != 4*numSamples || dest.data == NULL ||
			src.dataLength != 2*numSamples || src.data == NULL) {
		return false;
	}

	uint16_t* srcData = (uint16_t *)src.data;
	uint32_t* destData = (uint32_t *)dest.data;
	uint32_t tmp;
	for (uint64_t i = 0; i < numSamples; i++, srcData++, destData++) {
		tmp = *srcData;
		tmp = (tmp << 16) | tmp;
		*destData = tmp;
	}

	return true;
}

bool downSample16To8(Image& dest, const Image& src) {
	uint64_t numSamples = dest.height * dest.width *
			Image::numberOfChannels(dest.encoding);

	if (dest.dataLength != numSamples || dest.data == NULL ||
			src.dataLength != 2*numSamples || src.data == NULL) {
		return false;
	}

	uint16_t* srcData = (uint16_t *)src.data;
	uint8_t* destData = (uint8_t *)dest.data;
	for (uint64_t i = 0; i < numSamples; i++, srcData++, destData++) {
		*destData = *srcData >> 8;
	}

	return true;
}

bool downSample32To8(Image& dest, const Image& src) {
	uint64_t numSamples = dest.height * dest.width *
			Image::numberOfChannels(dest.encoding);

	if (dest.dataLength != numSamples || dest.data == NULL ||
			src.dataLength != 4*numSamples || src.data == NULL) {
		return false;
	}

	uint32_t* srcData = (uint32_t *)src.data;
	uint8_t* destData = (uint8_t *)dest.data;
	for (uint64_t i = 0; i < numSamples; i++, srcData++, destData++) {
		*destData = *srcData >> 24;
	}

	return true;
}

bool downSample32To16(Image& dest, const Image& src) {
	uint64_t numSamples = dest.height * dest.width *
			Image::numberOfChannels(dest.encoding);

	if (dest.dataLength != 2*numSamples || dest.data == NULL ||
			src.dataLength != 4*numSamples || src.data == NULL) {
		return false;
	}

	uint32_t* srcData = (uint32_t *)src.data;
	uint16_t* destData = (uint16_t *)dest.data;
	for (uint64_t i = 0; i < numSamples; i++, srcData++, destData++) {
		*destData = *srcData >> 16;
	}

	return true;
}

bool compress(Image& dest, const Image& src) {
	if (dest.encoding == Image::JPEG) {
		if (src.encoding == Image::RGB8) {
			int cSize = jpeg_compress((char *)dest.data, (const char *)src.data,
					src.width, src.height, dest.dataLength,
					CASROS_JPEG_COMPRESSION_QUALITY);
			if (cSize > (int)dest.dataLength)
				return false;
			dest.dataLength = cSize;
			return true;
		}

		ImagePtr rgb = src.inEncoding(Image::RGB8);
		if (!convertImage(*rgb, src))
			return false;
		return compress(dest, *rgb);
	}

	// TODO: Compress to PNG

	return false;
}

bool decompress(Image& dest, const Image& src) {
	if (src.encoding == Image::JPEG) {
		if (dest.encoding == Image::RGB8) {
			jpeg_decompress((unsigned char *)dest.data, dest.dataLength,
					(const unsigned char *)src.data, src.dataLength);
			return true;
		}

		ImagePtr rgb = src.inEncoding(Image::RGB8);
		if (!decompress(*rgb, src))
			return false;
		return convertImage(dest, *rgb);
	}

	// TODO: Decompress PNG

	return false;
}

template <typename T>
bool swapRandB(Image& dest, const Image& src) {
	uint64_t numPixels = dest.height * dest.width;
	int channels = Image::numberOfChannels(src.encoding);
	uint64_t minBuffer = numPixels * channels * sizeof(T);

	if (channels != Image::numberOfChannels(dest.encoding) ||
			dest.dataLength < minBuffer || src.dataLength < minBuffer) {
		return false;
	}

	const T* srcP = (const T *)src.data;
	T* destP = (T *)dest.data;

	if (channels == 4) {
		for (size_t p = 0; p < numPixels; p++, srcP += 4, destP += 4) {
			destP[0] = srcP[2];
			destP[1] = srcP[1];
			destP[2] = srcP[0];
			destP[3] = srcP[3];
		}
	} else if (channels == 3) {
		for (size_t p = 0; p < numPixels; p++, srcP += 3, destP += 3) {
			destP[0] = srcP[2];
			destP[1] = srcP[1];
			destP[2] = srcP[0];
		}
	} else {
		return false;
	}
	return true;
};

template <typename T>
bool addAlphas(Image& dest, const Image& src) {
	const T* inP = (const T*)src.data;
	T* outP = (T*)dest.data;

	uint64_t numPixels = dest.width*dest.height;
	uint32_t maxValue = (sizeof(T)==sizeof(uint16_t))?__UINT16_MAX__:__UINT8_MAX__;

	for (uint64_t p = 0; p < numPixels; p++, inP += 3, outP += 4) {
		outP[0] = inP[0];
		outP[1] = inP[1];
		outP[2] = inP[2];
		outP[3] = maxValue;
	}
	return true;
};

template <typename T>
bool dropAlphas(Image& dest, const Image& src) {
	const T* inP = (const T*)src.data;
	T* outP = (T*)dest.data;

	uint64_t numPixels = dest.width*dest.height;

	for (uint64_t p = 0; p < numPixels; p++, inP += 4, outP += 3) {
		outP[0] = inP[0];
		outP[1] = inP[1];
		outP[2] = inP[2];
	}
	return true;
};

#define BAYER_PIXEL(X,Y)		src[((Y)*width+(X))]
template <typename T>
bool decodeBayer(T* dest, const T* src, uint32_t height, uint32_t width, Image::Encoding enc) {
	unsigned int RED_ROW = 0, RED_COLUMN = 0;
	if (enc == Image::BAYER_BGGR8 || enc == Image::BAYER_BGGR16) {
		RED_ROW = 1; RED_COLUMN = 1;
	} else if (enc == Image::BAYER_GRBG8 || enc == Image::BAYER_GRBG16) {
		RED_ROW = 0; RED_COLUMN = 1;
	} else if (enc == Image::BAYER_GBRG8 || enc == Image::BAYER_GBRG16) {
		RED_ROW = 1; RED_COLUMN = 0;
	}
	uint32_t R, G, B, CNT;

	for (uint32_t i = 0; i < height; i++) {
		for (uint32_t j = 0; j < width; j++) {
			if (i%2 == RED_ROW) {
				if (j%2 == RED_COLUMN) {
					R = BAYER_PIXEL(j, i);

					G = CNT = 0;
					if (i > 0) {
						G += BAYER_PIXEL(j, i-1); CNT++;
					}
					if (i < height - 1) {
						G += BAYER_PIXEL(j, i+1); CNT++;
					}
					if (j > 0) {
						G += BAYER_PIXEL(j-1, i); CNT++;
					}
					if (j < width - 1) {
						G += BAYER_PIXEL(j+1, i); CNT++;
					}
					G /= CNT;

					B = CNT = 0;
					if (i > 0 && j > 0) {
						B += BAYER_PIXEL(j-1, i-1); CNT++;
					}
					if (i > 0 && j < width - 1) {
						B += BAYER_PIXEL(j+1, i-1); CNT++;
					}
					if (i < height - 1 && j > 0) {
						B += BAYER_PIXEL(j-1, i+1); CNT++;
					}
					if (i < height - 1 && j < width - 1) {
						B += BAYER_PIXEL(j+1, i+1); CNT++;
					}
					B /= CNT;
				} else {
					G = BAYER_PIXEL(j, i);

					R = CNT = 0;
					if (j > 0) {
						R += BAYER_PIXEL(j-1, i); CNT++;
					}
					if (j < width - 1) {
						R += BAYER_PIXEL(j+1, i); CNT++;
					}
					R /= CNT;

					B = CNT = 0;
					if (i > 0) {
						B += BAYER_PIXEL(j, i-1); CNT++;
					}
					if (i < height - 1) {
						B += BAYER_PIXEL(j, i+1); CNT++;
					}
					B /= CNT;
				}
			} else {
				if (j%2 == RED_COLUMN) {
					G = BAYER_PIXEL(j, i);

					R = CNT = 0;
					if (i > 0) {
						R += BAYER_PIXEL(j, i-1); CNT++;
					}
					if (i < height - 1) {
						R += BAYER_PIXEL(j, i+1); CNT++;
					}
					R /= CNT;


					B = CNT = 0;
					if (j > 0) {
						B += BAYER_PIXEL(j-1, i); CNT++;
					}
					if (j < width - 1) {
						B += BAYER_PIXEL(j+1, i); CNT++;
					}
					B /= CNT;
				} else {
					B = BAYER_PIXEL(j, i);

					R = CNT = 0;
					if (i > 0 && j > 0) {
						R += BAYER_PIXEL(j-1, i-1); CNT++;
					}
					if (i > 0 && j < width - 1) {
						R += BAYER_PIXEL(j+1, i-1); CNT++;
					}
					if (i < height - 1 && j > 0) {
						R += BAYER_PIXEL(j-1, i+1); CNT++;
					}
					if (i < height - 1 && j < width - 1) {
						R += BAYER_PIXEL(j+1, i+1); CNT++;
					}
					R /= CNT;

					G = CNT = 0;
					if (i > 0) {
						G += BAYER_PIXEL(j, i-1); CNT++;
					}
					if (i < height - 1) {
						G += BAYER_PIXEL(j, i+1); CNT++;
					}
					if (j > 0) {
						G += BAYER_PIXEL(j-1, i); CNT++;
					}
					if (j < width - 1) {
						G += BAYER_PIXEL(j+1, i); CNT++;
					}
					G /= CNT;
				}
			}

			dest[0] = R; dest[1] = G; dest[2] = B;
			dest += 3;
		}
	}
	return true;
};

bool decodeBayer(Image& dest, const Image& src) {
	if (Image::bitDepth(dest.encoding) == Image::bitDepth(src.encoding) &&
			(dest.encoding == Image::RGB8 || dest.encoding == Image::RGB16)) {
		if (dest.height <= 1 || dest.width <= 1)
			return false;
		// decode bayer to rgb

		if (Image::bitDepth(dest.encoding) == 16) {
			return decodeBayer<uint16_t>((uint16_t*)dest.data, (const uint16_t *)src.data,
					src.height, src.width, src.encoding);
		} else {
			return decodeBayer<uint8_t>((uint8_t*)dest.data, (const uint8_t*)src.data,
					src.height, src.width, src.encoding);
		}
	}
	ImagePtr rgb;
	if (Image::bitDepth(src.encoding) == 16) {
		rgb = new Image(Image::RGB16, dest.height, dest.width);
	} else {
		rgb = new Image(Image::RGB8, dest.height, dest.width);
	}
	if (!convertImage(*rgb, src)) {
		return false;
	}

	return convertImage(dest, *rgb);
}

bool encodeBayer(Image& dest, const Image& src) {
	if (Image::bitDepth(dest.encoding) == Image::bitDepth(src.encoding) &&
			(src.encoding == Image::RGB8 || src.encoding == Image::RGB16)) {
		// Encode Bayer
		int channel[2][2];

		if (dest.encoding == Image::BAYER_RGGB8 || dest.encoding == Image::BAYER_RGGB16) {
			channel[0][0] = 0; channel[0][1] = 1;
			channel[1][0] = 1; channel[1][1] = 2;
		} else if (dest.encoding == Image::BAYER_BGGR8 || dest.encoding == Image::BAYER_BGGR16) {
			channel[0][0] = 2; channel[0][1] = 1;
			channel[1][0] = 1; channel[1][1] = 0;
		} else if (dest.encoding == Image::BAYER_GRBG8 || dest.encoding == Image::BAYER_GRBG16) {
			channel[0][0] = 1; channel[0][1] = 0;
			channel[1][0] = 2; channel[1][1] = 1;
		} else {
			channel[0][0] = 1; channel[0][1] = 2;
			channel[1][0] = 0; channel[1][1] = 1;
		}

		if (Image::bitDepth(dest.encoding) == 16) {
			const uint16_t* srcP = (const uint16_t *)src.data;
			uint16_t* destP = (uint16_t *)dest.data;

			for (uint32_t i = 0; i < dest.height; i++) {
				for (uint32_t j = 0; j < dest.width; j++, destP++, srcP += 3) {
					*destP = srcP[channel[i%2][j%2]];
				}
			}
		} else {
			const uint8_t* srcP = (const uint8_t *)src.data;
			uint8_t* destP = (uint8_t *)dest.data;

			for (uint32_t i = 0; i < dest.height; i++) {
				for (uint32_t j = 0; j < dest.width; j++, destP++, srcP += 3) {
					*destP = srcP[channel[i%2][j%2]];
				}
			}
		}
		return true;
	}

	ImagePtr rgb;
	if (Image::bitDepth(dest.encoding) == 16) {
		rgb = new Image(Image::RGB16, dest.height, dest.width);
	} else {
		rgb = new Image(Image::RGB8, dest.height, dest.width);
	}
	if (!convertImage(*rgb, src)) {
		return false;
	}

	return convertImage(dest, *rgb);
}

bool convertYUV422toYUV420P(Image& dest, const Image& src) {
	unsigned int x, y, numPixels = dest.width * dest.height,
			Y1, Y2, Y3, Y4, U1, V1, U2, V2,
		numChromaLines = (dest.height + 1) / 2,
		numChromasPerLine = (dest.width + 1) / 2;

	const unsigned char *inL1 = (const unsigned char *)src.data;
	const unsigned char *inL2 = inL1 + numChromasPerLine * 4;

	unsigned char *outY1 = (unsigned char *)dest.data;
	unsigned char *outY2 = outY1 + dest.width;
	unsigned char *outU = outY1 + numPixels;
	unsigned char *outV = outU + numChromaLines * numChromasPerLine;

	for (y = 0; y < dest.height - 1; y += 2)
	{
		for (x = 0; x < dest.width - 1; x += 2)
		{
			Y1 = inL1[0];
			U1 = inL1[1];
			Y2 = inL1[2];
			V1 = inL1[3];
			Y3 = inL2[0];
			U2 = inL2[1];
			Y4 = inL2[2];
			V2 = inL2[3];

			outY1[0] = Y1;
			outY1[1] = Y2;
			outY2[0] = Y3;
			outY2[1] = Y4;

			outU[0] = (U1 + U2) >> 1; // (U1 + U2) / 2
			outV[0] = (V1 + V2) >> 1; // (V1 + V2) / 2

			inL1 += 4;
			inL2 += 4;

			outY1 += 2;
			outY2 += 2;
			outU++;
			outV++;
		}

		if (x < dest.width)
		{
			Y1 = inL1[0];
			U1 = inL1[1];
			Y2 = inL1[2];
			V1 = inL1[3];
			Y3 = inL2[0];
			U2 = inL2[1];
			Y4 = inL2[2];
			V2 = inL2[3];

			outY1[0] = Y1;
			outY2[0] = Y3;

			outU[0] = (U1 + U2) >> 1; // (U1 + U2) / 2
			outV[0] = (V1 + V2) >> 1; // (V1 + V2) / 2

			inL1 += 4;
			inL2 += 4;

			outY1 += 1;
			outY2 += 1;
			outU++;
			outV++;
		}

		inL1 += numChromasPerLine << 2; // (numChromasPerLine * 4)
		inL2 += numChromasPerLine << 2; // (numChromasPerLine * 4)

		outY1 += dest.width;
		outY2 += dest.width;
	}
	if (y < dest.height)
	{
		for (x = 0; x < dest.width - 1; x += 2)
		{
			Y1 = inL1[0];
			U1 = inL1[1];
			Y2 = inL1[2];
			V1 = inL1[3];

			outY1[0] = Y1;
			outY1[1] = Y2;

			outU[0] = U1;
			outV[0] = V1;

			inL1 += 4;

			outY1 += 2;
			outU++;
			outV++;
		}

		if (x < dest.width)
		{
			Y1 = inL1[0];
			U1 = inL1[1];
			Y2 = inL1[2];
			V1 = inL1[3];

			outY1[0] = Y1;

			outU[0] = U1;
			outV[0] = V1;

			inL1 += 4;

			outY1 += 1;
			outU++;
			outV++;
		}
	}
	return true;
}

bool convertYUV420PtoYUV422(Image& dest, const Image& src) {
	unsigned int x, y, numPixels = dest.width * dest.height,
			Y1, Y2, Y3, Y4, U, V,
		numChromaLines = (dest.height + 1) / 2,
		numChromasPerLine = (dest.width + 1) / 2;

	const unsigned char *inY1 = (const unsigned char *)src.data;
	const unsigned char *inY2 = inY1 + dest.width;
	const unsigned char *inU = inY1 + numPixels;
	const unsigned char *inV = inU + numChromaLines * numChromasPerLine;
	unsigned char *outL1 = (unsigned char *)dest.data;
	unsigned char *outL2 = outL1 + numChromasPerLine * 4;

	for (y = 0; y < dest.height - 1; y += 2)
	{
		for (x = 0; x < dest.width - 1; x += 2)
		{
			Y1 = inY1[0];
			Y2 = inY1[1];
			Y3 = inY2[0];
			Y4 = inY2[1];
			U  = inU[0];
			V  = inV[0];

			outL1[0] = Y1;
			outL1[1] = U;
			outL1[2] = Y2;
			outL1[3] = V;
			outL2[0] = Y3;
			outL2[1] = U;
			outL2[2] = Y4;
			outL2[3] = V;

			inY1 += 2;
			inY2 += 2;
			inU++;
			inV++;

			outL1 += 4;
			outL2 += 4;
		}

		if (x < dest.width)
		{
			Y1 = inY1[0];
			Y3 = inY2[0];
			U = inU[0];
			V = inV[0];

			outL1[0] = Y1;
			outL1[1] = U;
			outL1[2] = Y1;
			outL1[3] = V;
			outL2[0] = Y3;
			outL2[1] = U;
			outL2[2] = Y3;
			outL2[3] = V;

			inY1++;
			inY2++;
			inU++;
			inV++;
			outL1 += 4;
			outL2 += 4;
		}

		inY1 += dest.width;
		inY2 += dest.width;
		outL1 += numChromasPerLine << 2; // (numChromasPerLine * 4)
		outL2 += numChromasPerLine << 2; // (numChromasPerLine * 4)
	}

	if (y < dest.height)
	{
		for (x = 0; x < dest.width - 1; x += 2)
		{
			Y1 = inY1[0];
			Y2 = inY1[1];
			U  = inU[0];
			V  = inV[0];

			outL1[0] = Y1;
			outL1[1] = U;
			outL1[2] = Y2;
			outL1[3] = V;

			inY1 += 2;
			inU++;
			inV++;

			outL1 += 4;
		}

		if (x < dest.width)
		{
			Y1 = inY1[0];
			U = inU[0];
			V = inV[0];

			outL1[0] = Y1;
			outL1[1] = U;
			outL1[2] = Y1;
			outL1[3] = V;

			inY1++;
			inU++;
			inV++;
			outL1 += 4;
		}
	}

	return true;
}

bool convertRGB8toYUV422(Image& dest, const Image& src) {
	uint64_t x, y;
	int R1, G1, B1, R2, G2, B2, Y, U, V;
	const uint8_t *inPixels = (const uint8_t *)src.data;
	uint8_t *outPixels = (uint8_t *)dest.data;
	for (y = 0; y < src.height; y++) {
		for (x = 0; x < (src.width - 1); x += 2, inPixels += 6, outPixels += 4) {
			R1 = inPixels[0];
			G1 = inPixels[1];
			B1 = inPixels[2];
			R2 = inPixels[3];
			G2 = inPixels[4];
			B2 = inPixels[5];

			casros_convert_rgb_yuv(R1, G1, B1, &Y, &U, &V);
			outPixels[0] = Y;
			casros_convert_rgb_yuv(R2, G2, B2, &Y, &U, &V);
			outPixels[2] = Y;
			R1 = (R1 + R2) >> 1; // (R1 + R2) / 2
			G1 = (G1 + G2) >> 1; // (G1 + G2) / 2
			B1 = (B1 + B2) >> 1; // (B1 + B2) / 2
			casros_convert_rgb_yuv(R1, G1, B1, &Y, &U, &V);
			outPixels[1] = U;
			outPixels[3] = V;
		}

		if (x < src.width) {
			R1 = inPixels[0];
			G1 = inPixels[1];
			B1 = inPixels[2];
			casros_convert_rgb_yuv(R1, G1, B1, &Y, &U, &V);
			outPixels[0] = Y;
			outPixels[1] = U;
			outPixels[2] = Y;
			outPixels[3] = V;

			inPixels += 3;
			outPixels += 4;
		}
	}
	return true;
}

bool convertYUV422toRGB8(Image& dest, const Image& src) {
	unsigned int x, y;
	int Y1, U, Y2, V, R, G, B;
	const uint8_t *inPixels = (const uint8_t *)src.data;
	uint8_t *outPixels = (uint8_t *)dest.data;
	for (y = 0; y < src.height; y++) {
		for (x = 0; x < src.width - 1; x += 2, inPixels += 4, outPixels += 6) {
			Y1 = inPixels[0];
			U  = inPixels[1];
			Y2 = inPixels[2];
			V  = inPixels[3];

			casros_convert_yuv_rgb(Y1, U, V, &R, &G, &B);
			outPixels[0] = R;
			outPixels[1] = G;
			outPixels[2] = B;
			casros_convert_yuv_rgb(Y2, U, V, &R, &G, &B);
			outPixels[3] = R;
			outPixels[4] = G;
			outPixels[5] = B;
		}

		if (x < src.width) {
			Y1 = inPixels[0];
			U  = inPixels[1];
			Y2 = inPixels[2];
			V  = inPixels[3];

			casros_convert_yuv_rgb(Y1, U, V, &R, &G, &B);
			outPixels[0] = R;
			outPixels[1] = G;
			outPixels[2] = B;

			inPixels += 4;
			outPixels += 3;
		}
	}
	return true;
}

bool convertMono8toYUV422(Image& dest, const Image& src) {
	const uint8_t* srcL = (const uint8_t*)src.data;
	uint8_t* destL = (uint8_t*)dest.data;

	for (uint32_t i = 0; i < src.height; i++, srcL += src.step, destL += dest.step) {
		const uint8_t* srcP = srcL; uint8_t* destP = destL;
		for (uint32_t j = 0; j < (src.width - 1); j += 2, srcP += 2, destP += 4) {
			destP[0] = srcP[0];
			destP[1] = 0;
			destP[2] = srcP[1];
			destP[3] = 0;
		}
		if ((src.width % 2) == 1) {
			destP[0] = srcP[0];
			destP[1] = 0;
			destP[2] = 0;
			destP[3] = 0;
		}
	}
	return true;
}

bool convertYUV422toMono8(Image& dest, const Image& src) {
	const uint8_t* srcL = (const uint8_t*)src.data;
	uint8_t* destL = (uint8_t*)dest.data;

	for (uint32_t i = 0; i < src.height; i++, srcL += src.step, destL += dest.step) {
		const uint8_t* srcP = srcL; uint8_t* destP = destL;
		for (uint32_t j = 0; j < src.width; j++, destP++, srcP += 2) {
			*destP = *srcP;
		}
	}
	return true;
}

template <typename T>
bool convertRGBtoMono(Image& dest, const Image& src) {
	const T* srcP = (const T*)src.data;
	T* destP = (T*)dest.data;

	uint64_t numPixels = dest.width * dest.height;

	uint32_t tmp;
	for (uint64_t p = 0; p < numPixels; p++, destP++, srcP+=3) {
		tmp = 299*srcP[0];
		tmp += 587*srcP[1];
		tmp += 114*srcP[2];

		*destP = tmp/1000;
	}
	return true;
};

template <typename T>
bool convertMonotoRGB(Image& dest, const Image& src) {
	const T* srcP = (const T*)src.data;
	T* destP = (T*)dest.data;

	uint64_t numPixels = dest.width * dest.height;

	for (uint64_t p = 0; p < numPixels; p++, srcP++, destP+=3) {
		destP[0] = destP[1] = destP[2] = srcP[0];
	}
	return true;
};

bool convertImage(Image& dest, const Image& src) {
//	LOG("convertImage() - Converting encoding to %d from %d\n", dest.encoding, src.encoding);
	if (src.width != dest.width || src.height != dest.height)
		return false;
	if (dest.encoding == src.encoding) {
		if (dest.dataLength == src.dataLength && src.dataLength != 0) {
			memcpy(dest.data, src.data, src.dataLength);
			return true;
		}
		return false;
	} else if (src.dataLength == 0 || src.data == NULL) {
		return false;
	} else if (Image::isCompressed(dest.encoding)) {
		return compress(dest, src);
	} else if (Image::isCompressed(src.encoding)) {
		return decompress(dest, src);
	} else if (Image::isBayer(src.encoding)) {
		return decodeBayer(dest, src);
	} else if (Image::isBayer(dest.encoding)) {
		return encodeBayer(dest, src);
	} else if ((src.encoding == Image::Mono8 && dest.encoding == Image::Mono16) ||
			(src.encoding == Image::RGB8 && dest.encoding == Image::RGB16) ||
			(src.encoding == Image::RGBA8 && dest.encoding == Image::RGBA16) ||
			(src.encoding == Image::BGR8 && dest.encoding == Image::BGR16) ||
			(src.encoding == Image::BGRA8 && dest.encoding == Image::BGRA16) ||
			(src.encoding == Image::BAYER_RGGB8 && dest.encoding == Image::BAYER_RGGB16) ||
			(src.encoding == Image::BAYER_BGGR8 && dest.encoding == Image::BAYER_BGGR16) ||
			(src.encoding == Image::BAYER_GBRG8 && dest.encoding == Image::BAYER_GBRG16) ||
			(src.encoding == Image::BAYER_GRBG8 && dest.encoding == Image::BAYER_GRBG16)) {
		return upSample8To16(dest, src);
	} else if ((dest.encoding == Image::Mono8 && src.encoding == Image::Mono16) ||
			(dest.encoding == Image::RGB8 && src.encoding == Image::RGB16) ||
			(dest.encoding == Image::RGBA8 && src.encoding == Image::RGBA16) ||
			(dest.encoding == Image::BGR8 && src.encoding == Image::BGR16) ||
			(dest.encoding == Image::BGRA8 && src.encoding == Image::BGRA16) ||
			(dest.encoding == Image::BAYER_RGGB8 && src.encoding == Image::BAYER_RGGB16) ||
			(dest.encoding == Image::BAYER_BGGR8 && src.encoding == Image::BAYER_BGGR16) ||
			(dest.encoding == Image::BAYER_GBRG8 && src.encoding == Image::BAYER_GBRG16) ||
			(dest.encoding == Image::BAYER_GRBG8 && src.encoding == Image::BAYER_GRBG16)) {
		return downSample16To8(dest, src);
	} else if (src.encoding == Image::Mono8 && dest.encoding == Image::Mono32) {
		return upSample8To32(dest, src);
	} else if (dest.encoding == Image::Mono8 && src.encoding == Image::Mono32) {
		return downSample32To8(dest, src);
	} else if (src.encoding == Image::Mono16 && dest.encoding == Image::Mono32) {
		return upSample16To32(dest, src);
	} else if (dest.encoding == Image::Mono16 && src.encoding == Image::Mono32) {
		return downSample32To16(dest, src);
	} else if ((src.encoding == Image::RGB8 && dest.encoding == Image::BGR8) ||
			(src.encoding == Image::BGR8 && dest.encoding == Image::RGB8) ||
			(src.encoding == Image::RGBA8 && dest.encoding == Image::BGRA8) ||
			(src.encoding == Image::BGRA8 && dest.encoding == Image::RGBA8)) {
		return swapRandB<uint8_t>(dest, src);
	} else if ((src.encoding == Image::RGB16 && dest.encoding == Image::BGR16) ||
			(src.encoding == Image::BGR16 && dest.encoding == Image::RGB16) ||
			(src.encoding == Image::RGBA16 && dest.encoding == Image::BGRA16) ||
			(src.encoding == Image::BGRA16 && dest.encoding == Image::RGBA16)) {
		return swapRandB<uint16_t>(dest, src);
	} else if (dest.encoding == Image::YUV420P) {
		ImagePtr yuv422 = src.inEncoding(Image::YUV422);
		return convertYUV422toYUV420P(dest, *yuv422);
	} else if (src.encoding == Image::YUV420P) {
		if (dest.encoding == Image::YUV422)
			return convertYUV420PtoYUV422(dest, src);
		else {
			ImagePtr yuv422 = new Image(Image::YUV422, dest.height, dest.width);
			if (!convertYUV420PtoYUV422(*yuv422, src))
				return false;
			return convertImage(dest, *yuv422);
		}
	} else if (dest.encoding == Image::YUV422) {
		if (src.encoding == Image::Mono8) {
			return convertMono8toYUV422(dest, src);
		} else if (src.encoding == Image::Mono16 || src.encoding == Image::Mono32) {
			ImagePtr mono = src.inEncoding(Image::Mono8);
			return convertMono8toYUV422(dest, *mono);
		} else if (src.encoding == Image::RGB8) {
			return convertRGB8toYUV422(dest, src);
		}

		ImagePtr rgb = src.inEncoding(Image::RGB8);
		return convertRGB8toYUV422(dest, *rgb);
	} else if (src.encoding == Image::YUV422) {
		if (dest.encoding == Image::Mono8) {
			return convertYUV422toMono8(dest, src);
		} else if (dest.encoding == Image::Mono16 || dest.encoding == Image::Mono32) {
			ImagePtr mono = new Image(Image::Mono8, dest.height, dest.width);
			if (!convertYUV422toMono8(*mono, src))
				return false;
			return convertImage(dest, *mono);
		} else if (dest.encoding == Image::RGB8) {
			return convertYUV422toRGB8(dest, src);
		}
		ImagePtr rgb = new Image(Image::RGB8, dest.height, dest.width);
		if (!convertYUV422toRGB8(*rgb, src))
			return false;
		return convertImage(dest, *rgb);
	} else if (dest.encoding == Image::Mono8) {
		ImagePtr rgb = src.inEncoding(Image::RGB8);
		return convertRGBtoMono<uint8_t>(dest, *rgb);
	} else if (dest.encoding == Image::Mono16 || dest.encoding == Image::Mono32) {
		if (Image::bitDepth(src.encoding) == 8) {
			ImagePtr mono = src.inEncoding(Image::Mono8);
			return convertImage(dest, *mono);
		} else {
			ImagePtr rgb = src.inEncoding(Image::RGB16);
			if (dest.encoding == Image::Mono16) {
				return convertRGBtoMono<uint16_t>(dest, *rgb);
			}
			return convertImage(dest, *(rgb->inEncoding(Image::Mono16)));
		}
	} else if (src.encoding == Image::Mono8) {
		if (dest.encoding == Image::RGB8) {
			return convertMonotoRGB<uint8_t>(dest, src);
		}
		return convertImage(dest, *(src.inEncoding(Image::RGB8)));
	} else if (src.encoding == Image::Mono16 || src.encoding == Image::Mono32) {
		ImagePtr mono = src.inEncoding(Image::Mono16);
		if (dest.encoding == Image::RGB16) {
			return convertMonotoRGB<uint16_t>(dest, *mono);
		}
		return convertImage(dest, *(mono->inEncoding(Image::RGB16)));
	} else if ((src.encoding == Image::RGB8 && dest.encoding == Image::RGBA8) ||
			(src.encoding == Image::BGR8 && dest.encoding == Image::BGRA8)) {
		return addAlphas<uint8_t>(dest, src);
	} else if ((src.encoding == Image::RGB16 && dest.encoding == Image::RGBA16) ||
			(src.encoding == Image::BGR16 && dest.encoding == Image::BGRA16)) {
		return addAlphas<uint16_t>(dest, src);
	} else if ((dest.encoding == Image::RGB8 && src.encoding == Image::RGBA8) ||
			(dest.encoding == Image::BGR8 && src.encoding == Image::BGRA8)) {
		return dropAlphas<uint8_t>(dest, src);
	} else if ((dest.encoding == Image::RGB16 && src.encoding == Image::RGBA16) ||
			(dest.encoding == Image::BGR16 && src.encoding == Image::BGRA16)) {
		return dropAlphas<uint16_t>(dest, src);
	}

	if (Image::bitDepth(src.encoding) == 8 && src.encoding != Image::RGBA8 &&
			dest.encoding != Image::RGBA8) {
		ImagePtr rgba = src.inEncoding(Image::RGBA8);
		return convertImage(dest, *rgba);
	} else if (Image::bitDepth(src.encoding) == 16 && src.encoding != Image::RGBA16 &&
			dest.encoding != Image::RGBA16) {
		ImagePtr rgba = src.inEncoding(Image::RGBA16);
		return convertImage(dest, *rgba);
	} else if (Image::bitDepth(dest.encoding) == 8 && dest.encoding != Image::RGBA8 &&
			src.encoding != Image::RGBA8) {
		ImagePtr rgba = src.inEncoding(Image::RGBA8);
		return convertImage(dest, *rgba);
	} else if (Image::bitDepth(dest.encoding) == 16 && dest.encoding != Image::RGBA16 &&
			src.encoding != Image::RGBA16) {
		ImagePtr rgba = src.inEncoding(Image::RGBA16);
		return convertImage(dest, *rgba);
	}

	if (dest.encoding == Image::BGR8 || src.encoding == Image::BGR8) {
		ImagePtr rgb = src.inEncoding(Image::RGB8);
		return convertImage(dest, *rgb);
	} else if (dest.encoding == Image::BGR16 || src.encoding == Image::BGR16) {
		ImagePtr rgb = src.inEncoding(Image::RGB16);
		return convertImage(dest, *rgb);
	}

	ERROR("Image:: Cannot convert %d encoded images to %d.\n", src.encoding, dest.encoding);

	return false;
}

void writeJPEG(std::string filename, const Image& image) throw (IOException) {
	ImagePtr jpeg = image.inEncoding(Image::JPEG);
	FILE* file = fopen(filename.c_str(), "wb");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	size_t written = fwrite(jpeg->data, 1, jpeg->dataLength, file);
	fflush(file);
	fclose(file);

	if (written != jpeg->dataLength) {
		throw IOException::ErrorWritingFile(filename);
	}

	// Set file modification time to image.timestamp
	struct timeval times[2];
	times[0].tv_sec = times[1].tv_sec = image.getTimestamp().sec;
	times[0].tv_usec = times[1].tv_usec = image.getTimestamp().nsec/1000;
	utimes(filename.c_str(), times);
}

void writePNG(std::string filename, const Image& image) throw (IOException) {
	// TODO: writePNG()
}

#define BMP_HEADER_SIZE 54
void writeBMP(std::string filename, const Image& image) throw (IOException) {
	const char BMP_PADDING[4] = {0,0,0,0};

	ImagePtr bgr = image.inEncoding(Image::BGR8);

	unsigned char * pos;
	unsigned char * max;

	char header[BMP_HEADER_SIZE];
	unsigned int bytesPerRow=0, paddingPerRow=0, fileSize=0;
	int w = bgr->width, h = bgr->height;

	FILE *file = fopen(filename.c_str(), "wb");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	bytesPerRow = bgr->width*3;
	if (bytesPerRow%4 != 0) {// each row must be a multiple of four bytes
		paddingPerRow = 4 - (bytesPerRow % 4);
	}
	// prepare the file header
	fileSize = BMP_HEADER_SIZE + ((bytesPerRow+paddingPerRow) * bgr->height);
	memset(header, 0, BMP_HEADER_SIZE);
	header[0] = 'B';
	header[1] = 'M';
	header[2] = fileSize % 256;
	header[3] = (fileSize >> 8) % 256;
	header[4] = (fileSize >> 16) % 256;
	header[5] = (fileSize >> 24) % 256;
	header[10] = 54;
	header[14] = 40;
	header[18] =  w & 0x000000FF;
	header[19] = (w >> 8) & 0x000000FF;
	header[20] = (w >> 16) & 0x000000FF;
	header[21] = (w >> 24) & 0x000000FF;
	header[22] =  (-h) & 0x000000FF;
	header[23] = ((-h) >> 8) & 0x000000FF;
	header[24] = ((-h) >> 16) & 0x000000FF;
	header[25] = ((-h) >> 24) & 0x000000FF;
	header[26] = 1;
	header[28] = 24;
	if (fwrite(header, 1, BMP_HEADER_SIZE, file) != BMP_HEADER_SIZE) {
		fclose(file);
		throw IOException::ErrorWritingFile(filename);
	}

	if (paddingPerRow == 0) {
		if (fwrite(bgr->data, 1, bgr->dataLength, file) != bgr->dataLength) {
			fclose(file);
			throw IOException::ErrorWritingFile(filename);
		}
	} else {
		max = ((unsigned char*)bgr->data) + bgr->dataLength;
		for(pos = ((unsigned char*)bgr->data); pos < max; pos += bytesPerRow) {
			if (fwrite(pos, 1, bytesPerRow, file) != bytesPerRow) {
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
			if (fwrite(BMP_PADDING, 1, paddingPerRow, file) != paddingPerRow) {
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
		}
	}
	fflush(file);
	fclose(file);

	// Set file modification time to image.timestamp
	struct timeval times[2];
	times[0].tv_sec = times[1].tv_sec = image.getTimestamp().sec;
	times[0].tv_usec = times[1].tv_usec = image.getTimestamp().nsec/1000;
	utimes(filename.c_str(), times);
}

void writePGM(std::string filename, const Image& image) throw (IOException) {
	ImagePtr grey;
	if (Image::bitDepth(image.encoding) == 32)
		grey = image.inEncoding(Image::Mono32);
	else if (Image::bitDepth(image.encoding) == 16)
		grey = image.inEncoding(Image::Mono16);
	else
		grey = image.inEncoding(Image::Mono8);

	FILE *file = fopen(filename.c_str(), "wb");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	unsigned int numPixels = grey->width * grey->height;

	fprintf(file, "P5\n");

	char comments[256];
	sprintf(comments, "# timestamp: %d.%09d\n", image.getTimestamp().sec, image.getTimestamp().nsec);
	fprintf(file, "%s", comments);

	fprintf(file, "%u %u\n", grey->width, grey->height);

	if (grey->encoding == Image::Mono8) {
		fprintf(file, "255\n");
		if (fwrite(grey->data, 1, numPixels, file) != numPixels) {
			fflush(file);
			fclose(file);
			throw IOException::ErrorWritingFile(filename);
		}
	} else if (grey->encoding == Image::Mono16) {
		fprintf(file, "65535\n");

		const uint16_t *shorts = (const uint16_t *)grey->data;
		uint16_t nShorts[grey->width];
		for (uint32_t i = 0; i < grey->height; i++, shorts+=grey->width) {
			for (uint32_t j = 0; j < grey->width; j++) {
				nShorts[j] = htons(shorts[j]);
			}

			if (fwrite(nShorts, 2, grey->width, file) != grey->width) {
				fflush(file);
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
		}
	} else if (grey->encoding == Image::Mono32) {
		fprintf(file, "4294967295\n");

		const uint32_t *ints = (const uint32_t *)grey->data;
		uint32_t nInts[grey->width];
		for (uint32_t i = 0; i < grey->height; i++, ints +=grey->width) {
			for (uint32_t j = 0; j < grey->width; j++) {
				nInts[j] = htonl(ints[j]);
			}

			if (fwrite(nInts, 4, grey->width, file) != grey->width) {
				fflush(file);
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
		}
	}

	fflush(file);
	fclose(file);
}

void writePPM(std::string filename, const Image& image) throw (IOException) {
	ImagePtr rgb;
	if (Image::bitDepth(image.encoding) == 8)
		rgb = image.inEncoding(Image::RGB8);
	else
		rgb = image.inEncoding(Image::RGB16);

	FILE *file = fopen(filename.c_str(), "wb");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	unsigned int numPixels = rgb->width * rgb->height;

	fprintf(file, "P6\n");

	char comments[256];
	sprintf(comments, "# timestamp: %d.%09d\n", image.getTimestamp().sec, image.getTimestamp().nsec);
	fprintf(file, "%s", comments);
	fprintf(file, "%u %u\n", rgb->width, rgb->height);

	if (rgb->encoding == Image::RGB8) {
		fprintf(file, "255\n");
		if (fwrite(rgb->data, 3, numPixels, file) != numPixels) {
			fflush(file);
			fclose(file);
			throw IOException::ErrorWritingFile(filename);
		}
	} else if (rgb->encoding == Image::RGB16) {
		fprintf(file, "65535\n");

		const uint16_t * shorts = (const uint16_t *)rgb->data;
		uint32_t samplesPerRow = rgb->width*3;
		uint16_t nShorts[samplesPerRow];
		for (uint32_t i = 0; i < rgb->height; i++, shorts += samplesPerRow) {
			for (uint32_t j = 0; j < rgb->width; j++) {
				uint32_t J = 3*j;
				nShorts[J] = htons(shorts[J]);
				nShorts[J+1] = htons(shorts[J+1]);
				nShorts[J+2] = htons(shorts[J+2]);
			}

			if (fwrite(nShorts, 6, rgb->width, file) != rgb->width) {
				fflush(file);
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
		}
	}

	fflush(file);
	fclose(file);
}

#define TUPLETYPE_GREYSCALE		"GRAYSCALE"
#define TUPLETYPE_RGB			"RGB"
#define TUPLETYPE_BGR			"BGR"
#define TUPLETYPE_RGBA			"RGBA"
#define TUPLETYPE_BGRA			"BGRA"
#define TUPLETYPE_YUV422		"YUV422"
#define TUPLETYPE_YUV420P		"YUV420P"
#define TUPLETYPE_JPEG			"JPEG"
#define TUPLETYPE_PNG			"PNG"
#define TUPLETYPE_BAYER_RGGB	"BAYER_RGGB"
#define TUPLETYPE_BAYER_BGGR	"BAYER_BGGR"
#define TUPLETYPE_BAYER_GRBG	"BAYER_GRBG"
#define TUPLETYPE_BAYER_GBRG	"BAYER_GBRG"
#define TUPLETYPE_UNKNOWN		"UNKNOWN"

void writePAM(std::string filename, const Image& image) throw (IOException) {
	unsigned long maxVal = 255;
	const char *tplType = TUPLETYPE_UNKNOWN;
	int bitDepth = Image::bitDepth(image.encoding),
		channels = Image::numberOfChannels(image.encoding);

	if (bitDepth == 8 || bitDepth == 0) {
		maxVal = 255;
	} else if (bitDepth == 16) {
		maxVal = 65535;
	} else {
		maxVal = 4294967295LU;
	}

	switch (image.encoding) {
	case Image::Mono8: case Image::Mono16: case Image::Mono32:
		tplType = TUPLETYPE_GREYSCALE; break;
	case Image::RGB8: case Image::RGB16:
		tplType = TUPLETYPE_RGB; break;
	case Image::BGR8: case Image::BGR16:
		tplType = TUPLETYPE_BGR; break;
	case Image::RGBA8: case Image::RGBA16:
		tplType = TUPLETYPE_RGBA; break;
	case Image::BGRA8: case Image::BGRA16:
		tplType = TUPLETYPE_BGRA; break;
	case Image::YUV422:
		tplType = TUPLETYPE_YUV422; break;
	case Image::YUV420P:
		tplType = TUPLETYPE_YUV420P; break;
	case Image::JPEG:
		tplType = TUPLETYPE_JPEG; break;
	case Image::PNG:
		tplType = TUPLETYPE_PNG; break;
	case Image::BAYER_RGGB8: case Image::BAYER_RGGB16:
		tplType = TUPLETYPE_BAYER_RGGB; break;
	case Image::BAYER_BGGR8: case Image::BAYER_BGGR16:
		tplType = TUPLETYPE_BAYER_BGGR; break;
	case Image::BAYER_GRBG8: case Image::BAYER_GRBG16:
		tplType = TUPLETYPE_BAYER_GRBG; break;
	case Image::BAYER_GBRG8: case Image::BAYER_GBRG16:
		tplType = TUPLETYPE_BAYER_GBRG; break;
	case Image::Unknown:
	default:
		tplType = TUPLETYPE_UNKNOWN; break;
	}

	FILE *file = fopen(filename.c_str(), "wb");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	fprintf(file, "P7\n");

	char comments[256];
	sprintf(comments, "# timestamp: %d.%09d\n", image.getTimestamp().sec, image.getTimestamp().nsec);
	fprintf(file, "%s", comments);

	fprintf(file, "WIDTH %u\nHEIGHT %u\n", image.width, image.height);

	fprintf(file, "DEPTH %u\nMAXVAL %lu\nTUPLTYPE %s\n", channels, maxVal, tplType);

	fprintf(file, "ENDHDR\n");

	if (bitDepth == 16) {
		uint32_t shortsPerLine = image.step / sizeof(uint16_t);
		uint16_t nShorts[shortsPerLine + 1];
		const uint16_t *shorts;
		for (uint32_t i = 0; i < image.height; i++) {
			shorts = (const uint16_t *)(((const uint8_t *)image.data) + i*image.step);
			for (uint32_t j = 0; j < image.width; j++) {
				nShorts[j] = htons(shorts[j]);
			}

			if (fwrite(nShorts, 1, image.step, file) != image.step) {
				fflush(file);
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
		}
	} else if (bitDepth == 32) {
		uint32_t intsPerLine = image.step / sizeof(uint32_t);
		uint32_t nInts[intsPerLine + 1];
		const uint32_t *ints;
		for (uint32_t i = 0; i < image.height; i++) {
			ints = (const uint32_t *)(((const uint8_t *)image.data) + i*image.step);
			for (uint32_t j = 0; j < image.width; j++) {
				nInts[j] = htonl(ints[j]);
			}

			if (fwrite(nInts, 1, image.step, file) != image.step) {
				fflush(file);
				fclose(file);
				throw IOException::ErrorWritingFile(filename);
			}
		}
	} else {
		if (fwrite(image.data, 1, image.dataLength, file) != image.dataLength) {
			fflush(file);
			fclose(file);
			throw IOException::ErrorWritingFile(filename);
		}
	}

	fflush(file);
	fclose(file);
}

ImagePtr readJPEG(std::string filename) throw (IOException) {
	FILE *file = fopen(filename.c_str(), "rb");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	fseek(file, 0, SEEK_END);
	size_t dataSize = ftell(file);
	fseek(file, 0, SEEK_SET);

	ImagePtr rval = new Image(Image::JPEG, 0, 0);
	if (rval->data != NULL)
		free(rval->data);

	rval->data = malloc(dataSize);
	if (rval->data == NULL)
		return NULL;
	rval->dataLength = dataSize;

	size_t read = 0, readSum = 0;
	while (readSum < dataSize && (read = fread(((uint8_t *)rval->data) + readSum, 1, dataSize-readSum, file)) > 0) {
		readSum += read;
	}

	fclose(file);

	if (readSum != dataSize) {
		throw IOException::ErrorReadingFile(filename);
	}

	// get width and height
	if (jpeg_get_dimensions((uint8_t *)rval->data, rval->dataLength, (int *)&rval->width, (int *)&rval->height) != 0) {
		return NULL;
	}

	// Get file modification time for rval->timestamp
	struct stat fileInfo;
	int err = stat(filename.c_str(), &fileInfo);
	if (err == 0) {
		rval->timestamp = Time(fileInfo.st_mtim.tv_sec, fileInfo.st_mtim.tv_nsec);
	}
	return rval;
}

ImagePtr readPNG(std::string filename) throw (IOException) {
	// TODO: readPNG()
	return NULL;
}

ImagePtr readBMP(std::string filename) throw (IOException) {
	// TODO: readBMP()
	return NULL;
}

#define PBM_MAX_LINE_LEN	1024*4
bool getNextLinePBM(FILE *file, char *line) {
	if (fgets(line, PBM_MAX_LINE_LEN, file) == NULL)
		return false;
	while (line[0] == '#') {
		if (fgets(line, PBM_MAX_LINE_LEN, file) == NULL)
			return false;
	}
	return true;
}

bool getTimeStampPBM(FILE *file, char *line, Time* timestamp) {
	if (fgets(line, PBM_MAX_LINE_LEN, file) == NULL)
		return false;
	if (line[0] == '#') {
		sscanf(line, "# timestamp: %d.%d", &(timestamp->sec), &(timestamp->nsec));
		if (!getNextLinePBM(file, line))
			return false;
	}
	return true;
}

ImagePtr readPBM(std::string filename) throw (IOException) {
	bool binary = true;
	unsigned int channels = 1;
	unsigned long maxValue = 255;
	unsigned int width = 0, height = 0;
	Time timestamp = Time::now();
	Image::Encoding encoding = Image::Unknown;

	char line[PBM_MAX_LINE_LEN];
	FILE *file = fopen(filename.c_str(), "r");
	if (file == NULL) {
		throw IOException::UnableToOpenFile(filename);
	}

	if (!getNextLinePBM(file, line)) {
		fclose(file);
		throw IOException("Image: PBM Unable to read magic number for file " + filename);
	}

	if (strcasecmp(line, "P2\n") == 0 || strcasecmp(line, "P5\n") == 0) {
		// Read PGM header
		if (strcasecmp(line, "P2\n") == 0)
			binary = false;

		// read timestamp
		if (!getTimeStampPBM(file, line, &timestamp)) {
			fclose(file);
			throw IOException::ErrorReadingFile(filename);
		}

		if (sscanf(line, "%u %u", &width, &height) != 2) {
			DEBUG("Image: PBM Unable to parse dimensions %s\n", line);
			fclose(file);
			return NULL;
		}

		if (!getNextLinePBM(file, line)) {
			fclose(file);
			throw IOException("Image: PBM Unable to read max value for file " + filename);
		}

		if (sscanf(line, "%lu", &maxValue) != 1) {
			DEBUG("Image: PBM Unable to parse max value %s\n", line);
			fclose(file);
			return NULL;
		}

		if (maxValue == __UINT32_MAX__)
			encoding = Image::Mono32;
		else if (maxValue == __UINT16_MAX__)
			encoding = Image::Mono16;
		else if (maxValue == __UINT8_MAX__)
			encoding = Image::Mono8;
		else {
			DEBUG("Image: PBM Unable to process max value %lu\n", maxValue);
			fclose(file);
			return NULL;
		}
	} else if (strcasecmp(line, "P3\n") == 0 || strcasecmp(line, "P6\n") == 0) {
		// Read PPM header
		if (strcasecmp(line, "P3\n") == 0)
			binary = false;

		// read timestamp
		if (!getTimeStampPBM(file, line, &timestamp)) {
			fclose(file);
			throw IOException::ErrorReadingFile(filename);
		}

		if (sscanf(line, "%u %u", &width, &height) != 2) {
			DEBUG("Image: PBM Unable to parse dimensions %s\n", line);
			fclose(file);
			return NULL;
		}

		if (!getNextLinePBM(file, line)) {
			fclose(file);
			throw IOException("Image: PBM Unable to read max value for file " + filename);
		}

		if (sscanf(line, "%lu", &maxValue) != 1) {
			DEBUG("Image: PBM Unable to parse max value %s\n", line);
			fclose(file);
			return NULL;
		}

		if (maxValue == __UINT16_MAX__)
			encoding = Image::RGB16;
		else if (maxValue == __UINT8_MAX__)
			encoding = Image::RGB8;
		else {
			DEBUG("Image: PBM Unable to process max value %lu\n", maxValue);
			fclose(file);
			return NULL;
		}
	} else if (strcasecmp(line, "P7\n") == 0) {
		// Read PAM header
		char tuplType[PBM_MAX_LINE_LEN];

		while (strcasecmp(line, "ENDHDR\n") != 0) {
			if (fgets(line, PBM_MAX_LINE_LEN, file) == NULL) {
				fclose(file);
				throw IOException("Image: PBM Unable to read next line of header %s\n" + filename);
			}
			if (strcasecmp(line, "ENDHDR\n") != 0) {
				if (line[0] == '#') {
					int sec, nsec;
					if (sscanf(line, "# timestamp: %d.%d", &sec, &nsec) == 2) {
						timestamp.sec = sec; timestamp.nsec = nsec;
					}
				} else if (sscanf(line, "WIDTH %u", &width) == 1) {
				} else if (sscanf(line, "width %u", &width) == 1) {
				} else if (sscanf(line, "HEIGHT %u", &height) == 1) {
				} else if (sscanf(line, "height %u", &height) == 1) {
				} else if (sscanf(line, "DEPTH %u", &channels) == 1) {
				} else if (sscanf(line, "depth %u", &channels) == 1) {
				} else if (sscanf(line, "MAXVAL %lu", &maxValue) == 1) {
				} else if (sscanf(line, "maxval %lu", &maxValue) == 1) {
				} else if (sscanf(line, "TUPLTYPE %s", tuplType) == 1) {
				} else if (sscanf(line, "tupltype %s", tuplType) == 1) {
				} else {
					DEBUG("Image: PBM Invalid line of header in %s ignored.\n", filename.c_str());
				}
			}
		}
		if (strcasecmp(tuplType, TUPLETYPE_GREYSCALE) == 0) {
			if (channels == 1) {
				if (maxValue == __UINT8_MAX__) {
					encoding = Image::Mono8;
				} else if (maxValue == __UINT16_MAX__) {
					encoding = Image::Mono16;
				} else if (maxValue == __UINT32_MAX__) {
					encoding = Image::Mono32;
				}
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_RGB) == 0) {
			if (channels == 3 && maxValue == __UINT8_MAX__) {
				encoding = Image::RGB8;
			} else if (channels == 3 && maxValue == __UINT16_MAX__) {
				encoding = Image::RGB16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_BGR) == 0) {
			if (channels == 3 && maxValue == __UINT8_MAX__) {
				encoding = Image::BGR8;
			} else if (channels == 3 && maxValue == __UINT16_MAX__) {
				encoding = Image::BGR16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_RGBA) == 0) {
			if (channels == 4 && maxValue == __UINT8_MAX__) {
				encoding = Image::RGBA8;
			} else if (channels == 4 && maxValue == __UINT16_MAX__) {
				encoding = Image::RGBA16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_BGRA) == 0) {
			if (channels == 4 && maxValue == __UINT8_MAX__) {
				encoding = Image::BGRA8;
			} else if (channels == 4 && maxValue == __UINT16_MAX__) {
				encoding = Image::BGRA16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_BAYER_RGGB) == 0) {
			if (maxValue == __UINT8_MAX__) {
				encoding = Image::BAYER_RGGB8;
			} else if (maxValue == __UINT16_MAX__) {
				encoding = Image::BAYER_RGGB16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_BAYER_BGGR) == 0) {
			if (maxValue == __UINT8_MAX__) {
				encoding = Image::BAYER_BGGR8;
			} else if (maxValue == __UINT16_MAX__) {
				encoding = Image::BAYER_BGGR16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_BAYER_GRBG) == 0) {
			if (maxValue == __UINT8_MAX__) {
				encoding = Image::BAYER_GRBG8;
			} else if (maxValue == __UINT16_MAX__) {
				encoding = Image::BAYER_GRBG16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_BAYER_GBRG) == 0) {
			if (maxValue == __UINT8_MAX__) {
				encoding = Image::BAYER_GBRG8;
			} else if (maxValue == __UINT16_MAX__) {
				encoding = Image::BAYER_GBRG16;
			}
		} else if (strcasecmp(tuplType, TUPLETYPE_YUV422) == 0) {
			encoding = Image::YUV422;
		} else if (strcasecmp(tuplType, TUPLETYPE_YUV420P) == 0) {
			encoding = Image::YUV420P;
		} else if (strcasecmp(tuplType, TUPLETYPE_JPEG) == 0) {
			encoding = Image::JPEG;
		} else {
			DEBUG("Image: PBM Unknown tupltype \"%s\" for file %s.\n", tuplType, filename.c_str());
		}
	} else {
		DEBUG("Image: PBM Magic number %s unknown\n", line);
		fclose(file);
		return NULL;
	}

	ImagePtr rval = new Image(encoding, height, width);
	if (rval->data == NULL)
		return NULL;
	rval->timestamp = timestamp.toROS();

	if (!binary) {
		uint64_t numSamples = width * height * Image::numberOfChannels(encoding);

		uint32_t val;
		if (maxValue == __UINT32_MAX__) {
			uint32_t *ints = (uint32_t *)rval->data;
			for (uint64_t i = 0; i < numSamples; i++) {
				if (fscanf(file, "%u", &val) != 1) {
					fclose(file);
					throw IOException::ErrorReadingFile(filename);
				}
				ints[i] = val;
			}
		} else if (maxValue == __UINT16_MAX__) {
			uint16_t *shorts = (uint16_t *)rval->data;
			for (uint64_t i = 0; i < numSamples; i++) {
				if (fscanf(file, "%u", &val) != 1) {
					fclose(file);
					throw IOException::ErrorReadingFile(filename);
				}
				shorts[i] = val;
			}
		} else {
			uint16_t *bytes = (uint16_t *)rval->data;
			for (uint64_t i = 0; i < numSamples; i++) {
				if (fscanf(file, "%u", &val) != 1) {
					fclose(file);
					throw IOException::ErrorReadingFile(filename);
				}
				bytes[i] = val;
			}
		}
		return rval;
	}

	if (encoding == Image::JPEG || encoding == Image::PNG) {
		uint64_t dStart = ftell(file), dSize;
		fseek(file, 0, SEEK_END);
		dSize = ftell(file) - dStart;
		fseek(file, dStart, SEEK_SET);

		if (rval->dataLength < dSize) {
			free(rval->data);
			rval->data = malloc(dSize);
			if (rval->data == NULL) {
				fclose(file);
				return NULL;
			}
		}
		rval->dataLength = dSize;
		if (fread(rval->data, 1, dSize, file) != dSize) {
			fclose(file);
			throw IOException::ErrorReadingFile(filename);
		}
	} else if (maxValue == __UINT8_MAX__) {
		if (fread(rval->data, 1, rval->dataLength, file) != rval->dataLength) {
			fclose(file);
			throw IOException::ErrorReadingFile(filename);
		}
	} else if (maxValue == __UINT16_MAX__) {
		uint32_t shortsPerLine = rval->step / sizeof(uint16_t);

		uint16_t *line;
		for (uint32_t i = 0; i < rval->height; i++) {
			line = (uint16_t *)(((uint8_t *)rval->data) + i*rval->step);
			if (fread(line, 1, rval->step, file) != rval->step) {
				fclose(file);
				throw IOException::ErrorReadingFile(filename);
			}

			for (uint32_t j = 0; j < shortsPerLine; j++) {
				line[j] = ntohs(line[j]);
			}
		}
	} else if (maxValue == __UINT32_MAX__) {
		uint32_t intsPerLine = rval->step / sizeof(uint32_t);

		uint32_t *line;
		for (uint32_t i = 0; i < rval->height; i++) {
			line = (uint32_t *)(((uint8_t *)rval->data) + i*rval->step);
			if (fread(line, 1, rval->step, file) != rval->step) {
				fclose(file);
				throw IOException::ErrorReadingFile(filename);
			}

			for (uint32_t j = 0; j < intsPerLine; j++) {
				line[j] = ntohl(line[j]);
			}
		}
	}

	fclose(file);
	return rval;
}

} // namespace image

} // namespace crosbot
