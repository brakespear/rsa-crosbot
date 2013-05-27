/*
 * config.cpp
 *
 *  Created on: 06/12/2011
 *      Author: mmcgill
 */

#include <crosbot/config.hpp>
#include <libxml2/libxml/tree.h>

namespace crosbot {

class ConfigElement;
typedef Handle<ConfigElement> ConfigElementPtr;

class XMLDocElement : public HandledObject {
public:
	xmlDocPtr doc;
	XMLDocElement(xmlDocPtr doc) : doc(doc) {}
	~XMLDocElement() {
		if (doc != NULL)
			xmlFreeDoc(doc);
	}
};

typedef Handle<XMLDocElement> XMLDocElementPtr;

class XMLConfigElement : public ConfigElement {
public:
	XMLDocElementPtr doc;
	xmlNodePtr node;
	ConfigElementPtr parent;

	XMLConfigElement(XMLDocElementPtr doc, xmlNodePtr node, ConfigElementPtr parent=NULL) :
		doc(doc), node(node), parent(parent)
	{
		if (node != NULL)
			name = std::string((const char *) node->name);
	}

	bool hasParam(std::string paramName) {
		if (node == NULL)
			return false;
		return xmlHasProp(node, (const xmlChar *)paramName.c_str());
	}

	std::string getParam(std::string paramName, std::string defaultValue="") {
		if (node == NULL)
			return defaultValue;
		const xmlChar *xmlStr = xmlGetProp(node, (const xmlChar *)paramName.c_str());
		if (xmlStr == NULL)
			return defaultValue;
		return std::string((const char *)xmlStr);
	}

	unsigned int getChildCount() {
		if (node == NULL)
			return 0;
		int rval = 0;

		for (xmlNodePtr child = node->children; child != NULL; child = child->next) {
			if (child->type == XML_ELEMENT_NODE)
				++rval;
		}

		return rval;
	}

	ConfigElementPtr getChild(int idx) {
		for (xmlNodePtr child = node->children; child != NULL && idx >= 0; child = child->next) {
			if (child->type == XML_ELEMENT_NODE) {
				if (idx == 0) {
					return new XMLConfigElement(doc, child, this);
				}
				--idx;
			}
		}
		return NULL;
	}

	ConfigElementPtr getChild(std::string name) {
		for (xmlNodePtr child = node->children; child != NULL; child = child->next) {
			if (child->type == XML_ELEMENT_NODE) {
				if (strcasecmp(name.c_str(), (const char *)child->name) == 0) {
					return new XMLConfigElement(doc, child, this);
				}
			}
		}
		return NULL;
	}

	ConfigElementPtr getParent() {
		return parent;
	}
};

ConfigElementPtr ConfigElement::parseFile(std::string filename) {
	xmlDocPtr doc;
	xmlNodePtr node;

	doc = xmlParseFile(filename.c_str());
	if (doc == NULL) {
		return NULL;
	}

	XMLDocElementPtr docElem = new XMLDocElement(doc);

	for(node = doc->children; node != NULL; node = node->next)
	{
		if (node->type == XML_ELEMENT_NODE) {
			return new XMLConfigElement(docElem, node);
		}
	}

	return NULL;
}

} // namespace crosbot
