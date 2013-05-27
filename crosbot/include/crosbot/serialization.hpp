/*
 * serialization.h
 *
 *  Created on: 28/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_SERIALIZATION_H_
#define CROSBOT_SERIALIZATION_H_

#include <exception>
#include <string>
#include <vector>

#include <stdio.h>
#include <stdlib.h>

namespace crosbot {

class IOException : public std::exception {
	std::string description;
public:
	IOException() : description("IO Exception") {}
	IOException(const char *description) : description(description) {}
	IOException(const std::string& description) : description(description) {}
	~IOException() throw() {}

	const char* what() const throw() { return description.c_str(); }

	inline static IOException UnableToOpenFile(std::string filename) {
		std::string msg = "Unable to open file " + filename + ".";
		return IOException(msg);
	}

	inline static IOException ErrorWritingFile(std::string filename) {
		std::string msg = "Error when writing to file " + filename + ".";
		return IOException(msg);
	}

	inline static IOException ErrorReadingFile(std::string filename) {
		std::string msg = "Error when reading from file " + filename + ".";
		return IOException(msg);
	}

	inline static IOException OutOfMemory() {
		return IOException("Out of memory.");
	}
};

namespace serialization {

template <typename T>
class Serializer;

class OutputStream {
public:
	virtual ~OutputStream() {}

	template <typename T>
	inline size_t next(const T& n) throw (IOException) {
		Serializer<T> serializer;
		return serializer.write(n, *this);
	}

	virtual size_t write(const void *data, size_t bytes) throw (IOException)=0;
	virtual void seek(size_t pt) throw (IOException)=0;
	virtual size_t position() throw (IOException)=0;
};

class InputStream {
public:
	virtual ~InputStream() {}

	template <typename T>
	inline size_t next(T& n) throw (IOException) {
		Serializer<T> serializer;
		return serializer.read(n, *this);
	}

	virtual size_t read(void *data, size_t bytes) throw (IOException)=0;
	virtual void seek(size_t pt) throw (IOException)=0;
	virtual size_t position() throw (IOException)=0;
};

template <typename T>
class Serializer {
public:
	inline size_t write(const T&, OutputStream&) throw (IOException);
	inline size_t read(T&, InputStream&) throw (IOException);
	inline size_t serializedLength(const T&);
};

#define CASROS_SIMPLE_SERIALIZER(TYPE)													\
	template<> class Serializer<TYPE> {													\
	public:																				\
		inline size_t write(const TYPE& v, OutputStream& stream) throw (IOException) {	\
			size_t n = stream.write(&v, sizeof(TYPE));									\
			if (n != sizeof(TYPE))														\
				throw IOException("IOException: Out of space for write.");				\
			return n;																	\
		}																				\
		inline size_t read(TYPE& v, InputStream& stream) throw (IOException) {			\
			int n = stream.read(&v, sizeof(TYPE));										\
			if (n != sizeof(TYPE))														\
				throw IOException("IOException: Not enough space for read.");			\
			return n;																	\
		}																				\
		inline size_t serializedLength(const TYPE&) { return sizeof(TYPE); }			\
	};

CASROS_SIMPLE_SERIALIZER(signed char);
CASROS_SIMPLE_SERIALIZER(unsigned char);
CASROS_SIMPLE_SERIALIZER(signed short);
CASROS_SIMPLE_SERIALIZER(unsigned short);
CASROS_SIMPLE_SERIALIZER(signed int);
CASROS_SIMPLE_SERIALIZER(unsigned int);
CASROS_SIMPLE_SERIALIZER(signed long int);
CASROS_SIMPLE_SERIALIZER(unsigned long int);
CASROS_SIMPLE_SERIALIZER(signed long long int);
CASROS_SIMPLE_SERIALIZER(unsigned long long int);
CASROS_SIMPLE_SERIALIZER(float);
CASROS_SIMPLE_SERIALIZER(double);

template <typename T>
class Serializer< std::vector<T> > {
public:
	inline size_t write(const std::vector<const T>& v, OutputStream& stream) throw (IOException) {
		unsigned long int n = v.size();
		size_t rval = stream.next(n);

		Serializer<T> serializer;
		for (size_t s = 0; s < n; s++) {
			rval += serializer.write(v[s], stream);
		}
		return rval;
	}

	inline size_t read(std::vector<T>& v, InputStream& stream) throw (IOException) {
		unsigned long int n;
		size_t rval = stream.next(n);

		v.resize(n);
		Serializer<T> serializer;
		for (size_t s = 0; s < n; s++) {
			rval += serializer.read(v[s], stream);
		}
		return rval;
	}

	inline size_t serializedLength(const std::vector<const T>& v) {
		size_t rval = sizeof(unsigned long int);
		size_t n = v.size();
		Serializer<T> serializer;
		for (size_t s = 0; s < n; s++) {
			rval += serializer.serializedLength(v[s]);
		}
		return rval;
	}
};

template<>
class Serializer< std::string > {
public:
	inline size_t write(const std::string& v, OutputStream& stream) throw (IOException) {
		unsigned long int n = v.size();
		size_t rval = stream.next(n);
		if (stream.write(v.c_str(), v.size()) != n)
			throw IOException("IOException: Out of space for write.");
		return rval + n;
	}

	inline size_t read(std::string& v, InputStream& stream) throw (IOException) {
		unsigned long int n;
		size_t rval = stream.next(n);

		if (n > 0) {
			char cStr[n+1];

			if (stream.read(cStr, n) != n)
				throw IOException("IOException: Not enough space for read.");
			rval += n;
			cStr[n] = '\0';
			v = cStr;
		} else {
			v = "";
		}
		return rval;
	}

	inline size_t serializedLength(const std::string& v) {
		size_t rval = sizeof(unsigned long int) +  v.size();
		return rval;
	}
};

class FileOutputStream : public OutputStream {
	FILE *file;
	std::string filename;
public:
	FileOutputStream() : file(NULL) {}

	inline void open(std::string filename) throw(IOException) {
		file = fopen(filename.c_str(), "wb");
		if (file == NULL) {
			std::string desc = "IOException: Unable to open file " + filename + "for writing.";
			throw IOException(desc);
		}
		this->filename = filename;
	}

	inline void close() {
		if (file != NULL) {
			fclose(file);
			file = NULL;
		}
	}

	FileOutputStream(std::string filename) throw(IOException) : file(NULL) {
		open(filename);
	}

	~FileOutputStream() {
		close();
	}

	size_t write(const void *data, size_t bytes) throw (IOException) {
		if (file == NULL)
			throw IOException("IOException: No file opened.");
		size_t n = fwrite(data, 1, bytes, file);
		if (ferror(file))
			throw IOException("IOException: Error writing to file " + filename + "for writing.");
		return n;
	}

	void seek(size_t pt) throw (IOException) {
		if (file == NULL)
			throw IOException("IOException: No file opened.");
		fseek(file, pt, SEEK_SET);
		if (ferror(file))
			throw IOException("IOException: Error seeking in file " + filename + ".");
	}

	size_t position() throw (IOException) {
		if (file == NULL)
			return 0;
		size_t rval = ftell(file);
		return rval;
	}
};

class FileInputStream : public InputStream {
	FILE *file;
	std::string filename;
public:
	FileInputStream() : file(NULL) {}

	inline void open(std::string filename) throw(IOException) {
		file = fopen(filename.c_str(), "rb");
		if (file == NULL) {
			std::string desc = "IOException: Unable to open file " + filename + "for writing.";
			throw IOException(desc);
		}
		this->filename = filename;
	}

	inline void close() {
		if (file != NULL) {
			fclose(file);
			file = NULL;
		}
	}

	FileInputStream(std::string filename) throw(IOException) : file(NULL) {
		open(filename);
	}

	~FileInputStream() {
		close();
	}

	std::string getFilename() { return filename; }

	size_t read(void *data, size_t bytes) throw (IOException) {
		if (file == NULL)
			throw IOException("IOException: No file opened.");
		size_t n = fread(data, 1, bytes, file);
		if (ferror(file))
			throw IOException("IOException: Error reading from file " + filename + "for writing.");
		return n;
	}

	void seek(size_t pt) throw (IOException) {
		if (file == NULL)
			throw IOException("IOException: No file opened.");
		fseek(file, pt, SEEK_SET);
		if (ferror(file))
			throw IOException("IOException: Error seeking in file " + filename + ".");
	}

	size_t position() throw (IOException) {
		if (file == NULL)
			return 0;
		size_t rval = ftell(file);
		return rval;
	}
};

struct Buffer;
class BufferInputStream : public InputStream {
protected:
	Buffer& buffer;
	size_t posn;
public:
	BufferInputStream(Buffer& buffer) : buffer(buffer), posn(0) {}

	size_t read(void *data, size_t bytes) throw (IOException);
	void seek(size_t pt) throw (IOException);
	size_t position() throw (IOException) {
		return posn;
	}
};

class BufferOutputStream : public OutputStream {
protected:
	Buffer& buffer;
	size_t posn;
public:
	BufferOutputStream(Buffer& buffer) : buffer(buffer), posn(0) {}

	size_t write(const void *data, size_t bytes) throw (IOException);
	void seek(size_t pt) throw (IOException);
	size_t position() throw (IOException) {
		return posn;
	}
};

struct Buffer {
public:
	void *data;
	size_t bufferSize;

	Buffer() : data(NULL), bufferSize(0) {}
	Buffer(size_t size) throw (IOException) : bufferSize(0) {
		data = malloc(size);
		if (data == NULL)
			throw IOException("IOException: Out of memory.");
		bufferSize = size;
	}

	template <typename T>
	Buffer(T v) : bufferSize(0) {
		Serializer<T> serializer;
		size_t n = serializer.serializedLength();
		data = malloc(n);
		if (data == NULL)
			throw IOException("IOException: Out of memory.");
		bufferSize = n;
		BufferOutputStream bos(*this);
		serializer.write(v, bos);
	}
};

} // namespace serialization

} // namespace crosbot

#endif /* CROSBOT_SERIALIZATION_H_ */
