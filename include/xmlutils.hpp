/******************************************************************************/
/*!
 * @file
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 1 May 2020
 * @version 1.0.0
 * @brief Utility functions for libxml2 library
 ******************************************************************************/

#ifndef INCLUDE_XMLUTILS_HPP_
#define INCLUDE_XMLUTILS_HPP_

#include <libxml/xpath.h>
#include <string>

namespace xmlutils {

/**
 * @brief Parses an XML file
 * @param docname a path to a XML file
 * @return a pointer to a parsed XML document
 */
inline
xmlDocPtr getdoc(std::string docname) {
    xmlDocPtr doc;
    doc = xmlParseFile(docname.c_str());
    if (doc == NULL) {
        fprintf(stderr, "Document not parsed successfully. \n");
        return NULL;
    }
    return doc;
}

/**
 *
 * @param doc a pointer to a parsed XML document
 * @param xpath a search criteria
 * @return pointer to the xPath
 */
inline
xmlXPathObjectPtr getnodeset(xmlDocPtr doc, xmlChar* xpath) {
    xmlXPathContextPtr context;
    xmlXPathObjectPtr result;
    context = xmlXPathNewContext(doc);
    if (context == NULL) {
        printf("Error in xmlXPathNewContext\n");
        return NULL;
    }
    result = xmlXPathEvalExpression(xpath, context);
    xmlXPathFreeContext(context);

    if (result == NULL) {
        printf("Error in xmlXPathEvalExpression\n");
        return NULL;
    }

    if (xmlXPathNodeSetIsEmpty(result->nodesetval)) {
        xmlXPathFreeObject(result);
        //printf("No result\n");
        return NULL;
    }
    return result;
}

}
#endif /* INCLUDE_XMLUTILS_HPP_ */
