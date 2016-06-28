/*
 * xmldocconfig.hpp
 *
 *  Created on: 28/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_XMLDOCCONFIG_HPP_
#define CROSBOT_XMLDOCCONFIG_HPP_

#include <crosbot/config.hpp>

#include <libxml2/libxml/tree.h>

namespace crosbot {

/**
 * Memory managed wrapper around an xmlDocPtr object
 *     from libxml2
 */
class XMLDocElement : public HandledObject {

private:
    xmlDocPtr doc;

public:
    XMLDocElement(xmlDocPtr doc);
    virtual ~XMLDocElement();

};
typedef Handle<XMLDocElement> XMLDocElementPtr;

/**
 * Tree-based config structure, parsed from the legacy CASrobot XML configuration
 *    file format, using the libxml2 library.
 * Most methods (as required by the super class) provide wrappers around libxml2 function calls.
 */
class XMLConfigElement : public ConfigElement {

private:
    XMLDocElementPtr doc;
    xmlNodePtr node;
    ConfigElementPtr parent;

public:
    XMLConfigElement(XMLDocElementPtr doc, xmlNodePtr node, ConfigElementPtr parent = NULL);
    virtual ~XMLConfigElement() {};

    bool hasParam(std::string paramName);
    std::string getParam(std::string paramName, std::string defaultValue = "");
    unsigned int getChildCount();
    ConfigElementPtr getChild(int idx);
    ConfigElementPtr getChild(std::string name);
    ConfigElementPtr getParent();
};

} // namespace crosbot

#endif /* CROSBOT_XMLDOCCONFIG_HPP_ */
