/*
 * image.h
 *
 *  Created on: 01/03/2012
 *      Author: rescue
 */

#ifndef CROSBOT_IMAGE_H_
#define CROSBOT_IMAGE_H_

namespace crosbot {

namespace image {

bool convertImage(Image& dest, const Image& src);

void writeJPEG(std::string filename, const Image& image) throw (IOException);
void writePNG(std::string filename, const Image& image) throw (IOException);
void writeBMP(std::string filename, const Image& image) throw (IOException);
void writePGM(std::string filename, const Image& image) throw (IOException);
void writePPM(std::string filename, const Image& image) throw (IOException);
void writePAM(std::string filename, const Image& image) throw (IOException);

ImagePtr readJPEG(std::string filename) throw (IOException);
ImagePtr readPNG(std::string filename) throw (IOException);
ImagePtr readBMP(std::string filename) throw (IOException);
ImagePtr readPBM(std::string filename) throw (IOException);

} // namespace image

} // namespace crosbot

#endif /* CROSBOT_IMAGE_H_ */
