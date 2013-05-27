/* $Author: gerkey $
 * $Name: release-1-6-4 $
 * $Id: jpeg.h,v 1.2 2004/11/22 23:10:17 gerkey Exp $
 * $Source: /cvsroot/playerstage/code/player/client_libs/libplayerpacket/jpeg.h,v $
 * $Log: jpeg.h,v $
 * Revision 1.2  2004/11/22 23:10:17  gerkey
 * made libjpeg optional in libplayerpacket
 *
 * Revision 1.1  2004/09/17 18:09:05  inspectorg
 * *** empty log message ***
 *
 * Revision 1.1  2004/09/10 05:34:14  natepak
 * Added a JpegStream driver
 *
 * Revision 1.2  2003/12/30 20:49:49  srik
 * Added ifdef flags for compatibility
 *
 * Revision 1.1.1.1  2003/12/30 20:39:19  srik
 * Helicopter deployment with firewire camera
 *
 */

#ifndef CROSBOT_JPEG_H_
#define CROSBOT_JPEG_H_

namespace crosbot {

namespace image {

int 
jpeg_compress_YCbCr(char *dst, char *src, int width, int height, int dstsize, int quality);

int 
jpeg_compress(char *dst, const char *src, int width, int height, int dstsize, int quality);

void
jpeg_decompress(unsigned char *dst, int dst_size, const unsigned char *src, int src_size);

void
jpeg_decompress_from_file(unsigned char *dst, char *file, int size, int *width, int *height);

int
jpeg_get_dimensions(const unsigned char *src, int srcsize, int*width, int *height);

} // namespace image

} // namespace crosbot

#endif // CROSBOT_JPEG_H_
