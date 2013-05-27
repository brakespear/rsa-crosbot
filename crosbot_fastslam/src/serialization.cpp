/*
 * serialization.cpp
 *
 *  Created on: 28/02/2012
 *      Author: rescue
 */

#include <crosbot_fastslam/serialization.hpp>
#include <crosbot_fastslam/particle.hpp>

#define FASTSLAMMAP_LINELEN_MAX		4096
#define ACCEPTABLE_ERROR		0.00001
#define WITHIN_ERROR(V1, V2)		(V1 >= V2 - ACCEPTABLE_ERROR && V1 <= V2 + ACCEPTABLE_ERROR)

#define LOADED_FRAMEID_SENSOR		"sensor"
#define LOADED_FRAMEID_ROBOT		"base_link"
#define LOADED_FRAMEID_WORLD		"world"

namespace std {
std::string trim(const std::string str) {
	std::string rval = str;
	while (rval.size() > 1 && isspace(rval[0]))
		rval = rval.substr(1);
	while (rval.size() > 1 && isspace(rval[rval.size()-1]))
		rval = rval.substr(0, rval.size()-1);
	return rval;
}
} // namespace std

namespace casrobot {

// Values for the HeightMultiMap
unsigned int MULTIMAP_WIDTH			= 100;
unsigned int MULTIMAP_HEIGHT		= 100;
unsigned int MULTIMAP_SUB_WIDTH		= 100;
unsigned int MULTIMAP_SUB_HEIGHT	= 100;

struct ColouredPoint {
	// The X,Y,Z coordinates [m]
	double x, y, z;
	// The point's colour [RGB]
	double r, g, b;
};

struct HeightVal {
	float height; // height of the cell
	int views; // # of observations
	float prob; // occupancy log odds score
	int feature; // index of the near that cell
};

using namespace crosbot;

#define TIME_FORMAT		"%04d-%02d-%02d %02d:%02d:%02d.%03ld"
crosbot::Time parseTime(std::string time) {
	struct tm tinfo;
	long millis;
	tinfo.tm_wday = tinfo.tm_yday = 0;
	tinfo.tm_isdst = -1;
	sscanf(time.c_str(), TIME_FORMAT,
			&(tinfo.tm_year), &(tinfo.tm_mon), &(tinfo.tm_mday), &(tinfo.tm_hour), &(tinfo.tm_min), &(tinfo.tm_sec),
			&millis);
	tinfo.tm_year -= 1900;
	tinfo.tm_mon -= 1;

	return Time(mktime(&tinfo), millis * 1000000);
}

bool readPointCloud(std::string filename, Pose& robot, Pose& sensor, std::string& frameID, std::vector<Point>& points, std::vector<Colour>& colours) {
	Pose laser;
	FILE* file= fopen(filename.c_str(), "rb");
	if (file == NULL)
		return false;

	char timestamp[128+1];
	if (fscanf(file, "Timestamp: ") < 0 || fgets(timestamp, 128, file) == NULL) {
		ERROR("Logger: Error reading timestamp for pointcloud\n");
		fclose(file);
		return false;
	}

//	Time tstamp = parseTime(timestamp);

	double x, y, z, roll, pitch, yaw;

	if (fscanf(file, "Robot pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading robot pose for pointcloud.\n");
		return false;
	}
	robot = Pose(x,y,z,yaw,pitch,roll);

	if (fscanf(file, "Sensor pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading sensor pose for pointcloud.\n");
		return false;
	}
	sensor = Pose(x,y,z,yaw,pitch,roll);

	if (fscanf(file, "Abs sensor pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading absolute sensor pose for pointcloud.\n");
		fclose(file);
		return false;
	}
//	asp = Pose(x,y,z,yaw,pitch,roll);

	char cpf;
	unsigned long cpc;
	if (fscanf(file, "Coloured points: %c %lu\n", &cpf, &cpc) != 2) {
		ERROR("Logger: Error reading coloured point frame for pointcloud.\n");
		return NULL;
	}
	switch (cpf) {
	case 'w':
		frameID = LOADED_FRAMEID_WORLD; break;
	case 's':
		frameID = LOADED_FRAMEID_SENSOR; break;
	case 'r': default:
		// The default assumption is that loaded pointclouds are in robot coordinates.
		frameID = LOADED_FRAMEID_ROBOT; break;
	}
	if (cpc != 0) {
		casrobot::ColouredPoint cp;
		points.resize(cpc); colours.resize(cpc);
		for (unsigned long i = 0; i < cpc; ++i) {
			if (fread(&cp, 1, sizeof(casrobot::ColouredPoint), file) != sizeof(casrobot::ColouredPoint)) {
				ERROR("Logger: Error reading coloured points for pointcloud.\n");
				fclose(file);
				return false;
			}

			points[i].x = cp.x;
			points[i].y = cp.y;
			points[i].z = cp.z;
			colours[i].r = cp.r;
			colours[i].g = cp.g;
			colours[i].b = cp.b;
			colours[i].a = 255;
		}
	}

	Pose3D lp, alp;
	if (fscanf(file, "Laser pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading laser pose for pointcloud.\n");
		fclose(file);
		return false;
	}
	laser = Pose(x,y,z,yaw,pitch,roll);

	if (fscanf(file, "Abs laser pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading absolute laser pose for pointcloud.\n");
		fclose(file);
		return false;
	}
	alp = Pose(x,y,z,yaw,pitch,roll);

	char lpf;
	unsigned long lpc;
	if (fscanf(file, "Laser points: %c %lu\n", &lpf, &lpc) != 2) {
		ERROR("Logger: Error reading laser point frame for pointcloud.\n");
		fclose(file);
		return false;
	}
	if (lpc != 0 && cpc != 0) {
		LOG("crosbot_fastslam: %s: warn: File contains both coloured and uncoloured points. Uncoloured points discarded.\n",
				filename.c_str());
	} else if (lpc != 0) {
		sensor = laser;
		switch (lpf) {
		case 'w':
			frameID = LOADED_FRAMEID_WORLD; break;
		case 's':
			frameID = LOADED_FRAMEID_SENSOR; break;
		case 'r': default:
			// The default assumption is that loaded pointclouds are in robot coordinates.
			frameID = LOADED_FRAMEID_ROBOT; break;
		}

		points.resize(lpc); colours.clear();
		if (fread(&points[0], sizeof(Point3D), lpc, file) != lpc) {
			ERROR("Logger: Error reading laser points for pointcloud.\n");
			fclose(file);
			return false;
		}
	}
	return true;
}

MapCloudPtr loadPointCloud(std::string filename) {
	MapCloudPtr pc = new MapCloud();

	if (!readPointCloud(filename, pc->odometry, pc->sensor, pc->frameID, pc->cloud, pc->colours)) {
		return NULL;
	}

	if (pc->cloud.size() == 0)
		return NULL;

	return pc;
}

crosbot::ImagePtr loadImage(std::string filename) {
	ImagePtr image;
	try {
		image = Image::readFromFile(filename);
	} catch (IOException& e) {
		ERROR("Logger: Error reading image for snap.\n");
		return NULL;
	}
	return image;
}

crosbot::SnapPtr loadSnap(std::string filename) {
	FILE* file= fopen(filename.c_str(), "rb");
	if (file == NULL)
		return NULL;

	char timestamp[128];
	if (fscanf(file, "Timestamp: ") < 0 || fgets(timestamp, 128, file) == NULL) {
		ERROR("Logger: Error reading timestamp for snap.\n");
		fclose(file);
		return NULL;
	}
	SnapPtr rval = new Snap();

	rval->timestamp = parseTime(timestamp);


	double x, y, z, roll, pitch, yaw;

	rval->type = Snap::VICTIM;
	char cStatus;
	if (fscanf(file, "Type: victim Status: %c\n", &cStatus) != 1) {
		if(fscanf(file, "landmark Status: %c\n", &cStatus) == 1) {
			rval->type = Snap::LANDMARK;
		} else if (fscanf(file, "scan Status: %c\n", &cStatus) == 1) {
			rval->type = Snap::SCAN;
		} else {
			ERROR("Logger: Error reading type and status for snap.\n");
			fclose(file);
			return NULL;
		}
	}

	char line[FASTSLAMMAP_LINELEN_MAX+1];
	unsigned long len;

	if (fgets(line, FASTSLAMMAP_LINELEN_MAX, file) != line ||
			sscanf(line, "Description: (%lu)", &len) != 1) {
		ERROR("Logger: Error reading snap description.\n");
		fclose(file);
		return NULL;
	}
	std::string desc = line + (strlen(line) - (len + 1));
	rval->description = trim(desc);

	switch(cStatus) {
	case 'c':
		rval->status = Snap::CONFIRMED; break;
	case 'r':
		rval->status = Snap::REJECTED; break;
	case 'd':
		rval->status = Snap::DUPLICATE; break;
	case 'u': default:
		rval->status = Snap::UNCONFIRMED; break;
	}

	if (fscanf(file, "Robot pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading robot pose for snap.\n");
		fclose(file);
		return NULL;
	}
	rval->robot = Pose(x,y,z,yaw,pitch,roll);

	if (fscanf(file, "Target pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw) != 6) {
		ERROR("Logger: Error reading target pose for snap.\n");
		fclose(file);
		return NULL;
	}
	rval->pose = Pose(x,y,z,yaw,pitch,roll);

	std::string snapDir = filename;
	size_t f = snapDir.find_last_of('/');
	if (f == snapDir.npos)
		snapDir = "";
	else
		snapDir = snapDir.substr(0, f+1);
	snapDir = trim(snapDir);

	std::string dataFile;

	if (fgets(line, FASTSLAMMAP_LINELEN_MAX, file) == NULL) {
		if (!feof(file))
			ERROR("Logger: Error reading images in snap.\n");
		fclose(file);
		return rval;
	}

	while (sscanf(line, "image: (%lu)", &len) == 1) {
		// read in the image.
		dataFile = line + (strlen(line) - (len+1));
		dataFile = snapDir + trim(dataFile);
		ImagePtr image = loadImage(dataFile);
		if (image != NULL)
			rval->images.push_back(image);

		if (fgets(line, FASTSLAMMAP_LINELEN_MAX, file) == NULL) {
			if (!feof(file))
				ERROR("Logger: Error reading images in snap.\n");
			fclose(file);
			return rval;
		}
	}

	while (sscanf(line, "pointcloud: (%lu)", &len) == 1) {
		// read in the pointcloud.
		// read in the image.
		dataFile = line + (strlen(line) - (len+1));
		dataFile = snapDir + trim(dataFile);

		MapCloudPtr pc = loadPointCloud(dataFile);

		if (pc != NULL)
			rval->clouds.push_back(pc);
		if (fgets(line, FASTSLAMMAP_LINELEN_MAX, file) == NULL) {
			if (!feof(file))
				ERROR("Logger: Error reading clouds in snap.\n");
			fclose(file);
			return rval;
		}
	}

	return rval;
}

} // namespace casrobot

namespace crosbot {
using namespace fastslam;

namespace serialization {

const uint16_t Serializer<Particle>::version = 3;
const uint16_t Serializer<Particle>::subVersion = 1;

btTransform CASRobotPoseCorrection = Pose(0,0,0,DEG2RAD(90),0,0).getTransform();

size_t Serializer_gets(char* dest, InputStream& stream, unsigned int maxLen) {
	char c = ' ';
	size_t rval = 0, n;
	if (maxLen == 0)
		return 0;

	--maxLen;
	while (rval < maxLen && c != '\n' && c != '\r' && c != '\0') {
		n = stream.read(&c, 1);
		if (n < 1)
			break;
		dest[rval++] = c;
	}
	dest[rval] = '\0';

//	LOG("Serializer_gets: \"%s\"\n", dest);
	return rval;
}

#define FASTSLAMMAP_VERSION			2
#define FASTSLAMMAP_SUBVERSION		1
#define FASTSLAMMAP_SUBSUBVERSION	0
size_t Serializer<Particle>::readPreviousVersion(Particle& p, InputStream& stream)
		throw (IOException) {
	FileInputStream* fis = dynamic_cast<FileInputStream*>(&stream);
	std::string mapDirectory;
	if (fis != NULL)
		mapDirectory = fis->getFilename();
	size_t f = mapDirectory.find_last_of('/');
	if (f == mapDirectory.npos)
		mapDirectory = "";
	else
		mapDirectory = mapDirectory.substr(0, f+1);
	mapDirectory = trim(mapDirectory);

	uint8_t byte;
	size_t rval = stream.read(&byte, 1);
	if (byte == 0) {

		return 0;
	}

	unsigned int heightValSize = sizeof(casrobot::HeightVal), c, cols, rows, i, j;
	float cellWidth, cellHeight;
	double x, y, z, roll, pitch, yaw, width, height;
	unsigned long lng; char ch;

	/*
	 * A Mixed ASCII/binary file from CASRobot
	 */
	stream.seek(0);
	char line[FASTSLAMMAP_LINELEN_MAX+1];
	rval = Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);

	int mapVersion, mapSubVersion, mapSubSubVersion;
	if (sscanf(line, "FastSLAMMap v%d.%d.%d\n", &mapVersion, &mapSubVersion, &mapSubSubVersion) != 3) {
		if (sscanf(line, "SlamMap v%d.%d.%d\n", &mapVersion, &mapSubVersion, &mapSubSubVersion) != 3) {
			ERROR("FastSLAMMap: Unable to read map version information.\n");
			goto PARSE_ERROR;
		}
	}

	if (mapVersion != FASTSLAMMAP_VERSION || mapSubVersion != FASTSLAMMAP_SUBVERSION ||
			mapSubSubVersion != FASTSLAMMAP_SUBSUBVERSION) {
		ERROR("FastSLAMMap: Unsupported map version %d.%d.%d.\n",
				mapVersion, mapSubVersion, mapSubSubVersion);
		goto PARSE_ERROR;
	}

	rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
	if (sscanf(line, "Name: (%lu)", &lng) != 1) {
		ERROR("FastSLAMMap: Unable to read map name.\n");
		goto PARSE_ERROR;
	}

	rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
	if (sscanf(line, "Cell size: width=%fm height=%fm memory=%uB\n", &cellWidth, &cellHeight, &heightValSize) != 3) {
		ERROR("FastSLAMMap: Unable to read cell size.\n");
		goto PARSE_ERROR;
	} else if (!WITHIN_ERROR(cellWidth, CELL_WIDTH) || !WITHIN_ERROR(cellHeight, CELL_HEIGHT)) {
		ERROR("FastSLAMMap: Cell size in saved map not compatible with current map.\n");
		goto PARSE_ERROR;
	} else if (heightValSize != sizeof(HeightVal)) {
		ERROR("FastSLAMMap: HeightVal has changes so the map file format needs to be incremented\n");
		goto PARSE_ERROR;
	}

	// Zero map
	p.pose = Pose();
	p.trackerPose = Pose();
	p.parameters.mapRows = casrobot::MULTIMAP_HEIGHT;
	p.parameters.mapColumns = casrobot::MULTIMAP_HEIGHT;
	p.parameters.patchRows = casrobot::MULTIMAP_SUB_HEIGHT;
	p.parameters.patchColumns = casrobot::MULTIMAP_SUB_WIDTH;
	p.map = new HeightMultiMap(p.parameters);
	p.tags = new Map::TagList();
	p.history.clear();

	p.poseTransform = p.pose.getTransform();
	p.weight = 1;
	p.motionCloud = NULL;

	rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
	c = sscanf(line, "Current pose: {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
			&x, &y, &z, &roll, &pitch, &yaw);
	if (c == 0 && strcmp(line, "Map is empty.\n") == 0) {
		return rval;
	} else if (c != 6) {
		ERROR("FastSLAMMap: Unable to read current robot position.\n");
		goto PARSE_ERROR;
	}
	p.pose = Pose(x, y, z, yaw, pitch, roll).getTransform() * CASRobotPoseCorrection;

	rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
	c = sscanf(line, "HMM : offx=%lfm offy=%lfm cols=%u rows=%u\n",
			&x, &y, &cols, &rows);
	if (c == 0 && strcmp(line, "Map is empty.\n") == 0) {
		return rval;
	} else if (c != 4) {
		ERROR("FastSLAMMap: Unable to read height multimap info.\n");
		goto PARSE_ERROR;
	}

	p.parameters.mapRows = rows;
	p.parameters.mapColumns = cols;
	p.map = new HeightMultiMap(p.parameters);

	do {
		rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);

		c = sscanf(line, "HM(%u,%u): cols=%u rows=%u xorig=%lfm yorig=%lfm width=%lfm height=%lfm\n",
		                                &j, &i, &cols, &rows, &x, &y, &width, &height);
		if (c == 0) {
	        c = sscanf(line, "M(%u,%u): cols=%u rows=%u xorig=%lfm yorig=%lfm width=%lfm height=%lfm\n",
	                                        &j, &i, &cols, &rows, &x, &y, &width, &height);
		}

		if (c == 8) {
			HeightMapPtr hm = new HeightMap(rows, cols, p.parameters.initialOccupiedOdds);
			p.map->patches[i][j] = hm;

			casrobot::HeightVal hv;

			hm->width = width;
			hm->height = height;
			hm->xOrig = x;
			hm->yOrig = y;

//			p.parameters.patchColumns = p.map->patchColumns = cols;
//			p.parameters.patchRows = p.map->patchRows = rows;

			unsigned int cellCount = cols * rows;
			for (i = 0; i < cellCount; i++) {
				if (stream.read(&hv, sizeof(casrobot::HeightVal)) != sizeof(casrobot::HeightVal)) {
					ERROR("FastSLAMMap: Error reading casrobot::HeightVal.\n");
					goto PARSE_ERROR;
				}

				HeightVal& hmhv = hm->hlist[i];
				hmhv.height = hv.height;
				hmhv.prob = hv.prob;
				if (hv.views > 0) {
					hmhv.observations = 1;
					if (hv.views > 1) {
						hmhv.searches = hv.views-1;
					}
				}
			}
		}
	} while (c == 8);


	// read history
	if (strcmp(line, "history:\n") == 0) {
		rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);

		while ((c = sscanf(line, "Previous pose: o=%c {%lf, %lf, %lf}, {%lf, %lf, %lf}\n",
				&ch, &x, &y, &z, &roll, &pitch, &yaw)) == 7) {
			Particle::History hist(Pose(x, y, z, yaw, pitch, roll));
			hist.restart = tolower(ch) == 'y';
			hist.pose = hist.pose.getTransform() * CASRobotPoseCorrection;

			rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
			while (sscanf(line, "cloud: (%lu)", &lng) == 1) {
				std::string cloudFile = line + (strlen(line) - (lng + 1));
				cloudFile = mapDirectory + trim(cloudFile);
				hist.cloud = casrobot::loadPointCloud(cloudFile);
				rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
			}

			p.history.push_back(hist);
		}
	}

	// read snaps:
	if (strcmp(line, "snaps:\n") == 0) {
		Map::TagListPtr snaps = new Map::TagList();

		rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
		Pose3D srp, stp;
		double x2, y2, z2, roll2, pitch2, yaw2;
		while ((c = sscanf(line, "snap: v=%c rp={{%lf, %lf, %lf}{%lf, %lf, %lf}} tp={{%lf, %lf, %lf}{%lf, %lf, %lf}} (%lu)",
				&ch, &x, &y, &z, &roll, &pitch, &yaw,
				&x2, &y2, &z2, &roll2, &pitch2, &yaw2, &lng)) == 14) {
			std::string snapFile = line + (strlen(line) - (lng + 1));
			snapFile = mapDirectory + trim(snapFile);
			SnapPtr snap = casrobot::loadSnap(snapFile);

			Map::TagLocation tl(snap, Pose(x,y,z,yaw,pitch,roll),
					Pose(x2,y2,z2,yaw2,pitch2,yaw2));
			snaps->tags.push_back(tl);
			rval += Serializer_gets(line, stream, FASTSLAMMAP_LINELEN_MAX);
		}

		p.tags = snaps;
	}

	return rval;

PARSE_ERROR:
	stream.seek(0);
	return 0;
}

} // namespace serialization

} // namespace crosbot
