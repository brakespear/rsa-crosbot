/*
 * image.h
 *
 *  Created on: 22/03/2012
 *      Author: rescue
 */

#ifndef CROSBOT_RENDER_PRESET_H_
#define CROSBOT_RENDER_PRESET_H_

#include <crosbot_ui/panels/robot.hpp>

namespace crosbot {

namespace gui {

#define RENDER_PRESET	"preset"
#define RENDER_EMU		"emu"

#define VALID_PRESET(P)		((P) != INFINITY && (P) != -INFINITY && (P) != NAN)

class PresetRender : public RobotRender {
public:
	PresetRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void render();
    virtual void renderRobot(std::string label, std::vector<float> configuration, bool renderGoals = false)=0;
    virtual std::vector<float> getConfiguration()=0;
    virtual void setConfiguration(std::vector<float>)=0;

	virtual bool keyPressEvent(QKeyEvent *ke);
protected:
	bool renderPresets;
	int layout;

	std::vector< std::vector<float> > presets;
};

class EmuRender : public PresetRender {
protected:
	GLuint displayLists;
	bool displayListsSet;
public:
	EmuRender(RobotPanel& panel, ConfigElementPtr config);
	void start() {}
	void stop() {}

	void renderRobot(std::string label, std::vector<float> configuration, bool renderGoals = false);
	std::vector<float> getConfiguration();
	void setConfiguration(std::vector<float>);
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_RENDER_PRESET_H_ */
