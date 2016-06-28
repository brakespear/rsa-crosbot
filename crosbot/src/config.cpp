/*
 * config.cpp
 *
 *  Created on: 06/12/2011
 *      Author: mmcgill
 */

#include <ros/ros.h>

#include <crosbot/config.hpp>
#include <crosbot/config/rosconfig.hpp>
#include <crosbot/config/xmldocconfig.hpp>

namespace crosbot {

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

#ifdef ROS_VERSION

ROSConfigElement::ROSConfigElement(std::string name, ROSConfigElementPtr parent) :
    ROSConfigElement(ros::NodeHandle(name), parent)
{}

ROSConfigElement::ROSConfigElement(ros::NodeHandle nh, ROSConfigElementPtr parent) :
    nh(nh),
    parent(parent)
{
    this->nh = nh;
    this->parent = parent;
    resolvedNamespace = nh.getNamespace();

    // Set unresolved name
    int token = resolvedNamespace.find_last_of("/");
    childName = resolvedNamespace.substr(token+1, resolvedNamespace.npos);
}

unsigned int ROSConfigElement::getChildCount() {
    ROS_ERROR("ROS does not support returning a count of 'children' namespace(s), for");
    ROS_ERROR("ROSConfigElement::getChildCount() called on namespace '%s'.", resolvedNamespace.c_str());
    return 0;
}

ConfigElementPtr ROSConfigElement::getChild(int idx) {
    ROS_ERROR("ROS does not support returning numerical indexing of 'children' namespace(s), for");
    ROS_ERROR("ROSConfigElement::getChild() called on namespace '%s'.", resolvedNamespace.c_str());
    return NULL;
}

ConfigElementPtr ROSConfigElement::getChild(std::string name) {
    try {
        ros::NodeHandle ch(nh, name);
        return new ROSConfigElement(ch, this);
    } catch (ros::InvalidNameException& ine) {
        ROS_ERROR("Invalid 'child' namespace(s) '%s', for", name.c_str());
        ROS_ERROR("ROSConfigElement::getChild() called on namespace '%s'.", resolvedNamespace.c_str());
        ROS_ERROR("%s", ine.what());
    } catch (std::exception& ex) {
        ROS_ERROR("Error calling ROSConfigElement::getChild() called on namespace '%s'.", resolvedNamespace.c_str());
        ROS_ERROR("%s", ex.what());
    } catch (...) {
        ROS_ERROR("Unknown error calling ROSConfigElement::getChild() called on namespace '%s'.", resolvedNamespace.c_str());
    }
    return NULL;
}

#endif // ROS_VERSION

XMLDocElement::XMLDocElement(xmlDocPtr doc) :
    doc(doc)
{}

XMLDocElement::~XMLDocElement() {
    if (doc != NULL) {
        xmlFreeDoc(doc);
    }
}

XMLConfigElement::XMLConfigElement(XMLDocElementPtr doc, xmlNodePtr node, ConfigElementPtr parent) :
    doc(doc), node(node), parent(parent)
{
    if (node != NULL) {
        childName = std::string((const char *) node->name);

        // Set resolved namespace
        resolvedNamespace = "";
        if (parent != NULL) {
            resolvedNamespace = parent->getResolvedNamespace();
        }
        resolvedNamespace = resolvedNamespace + "/" + childName;
    }
}

bool XMLConfigElement::hasParam(std::string paramName) {
    if (node == NULL) {
        return false;
    }
    return xmlHasProp(node, (const xmlChar *)paramName.c_str());
}

std::string XMLConfigElement::getParam(std::string paramName, std::string defaultValue) {
    if (node == NULL) {
        return defaultValue;
    }

    const xmlChar *xmlStr = xmlGetProp(node, (const xmlChar *)paramName.c_str());
    if (xmlStr == NULL) {
        return defaultValue;
    }

    return std::string((const char *)xmlStr);
}

unsigned int XMLConfigElement::getChildCount() {
    if (node == NULL) {
        return 0;
    }
    int rval = 0;

    for (xmlNodePtr child = node->children; child != NULL; child = child->next) {
        if (child->type == XML_ELEMENT_NODE)
            ++rval;
    }

    return rval;
}

ConfigElementPtr XMLConfigElement::getChild(int idx) {
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

ConfigElementPtr XMLConfigElement::getChild(std::string name) {
    for (xmlNodePtr child = node->children; child != NULL; child = child->next) {
        if (child->type == XML_ELEMENT_NODE) {
            if (strcasecmp(name.c_str(), (const char *)child->name) == 0) {
                return new XMLConfigElement(doc, child, this);
            }
        }
    }
    return NULL;
}

ConfigElementPtr XMLConfigElement::getParent() {
    return parent;
}

} // namespace crosbot
