//
// Fake libxslt/transform.h — see xsltInternals.h in this directory.
// Provides the xslt entry points that rl::xml::Stylesheet compiles against,
// as no-op stubs (the stylesheet code paths are never executed for rlmdl).
//

#ifndef LIBXSLT_TRANSFORM_H
#define LIBXSLT_TRANSFORM_H

#include "xsltInternals.h"
#include <libxml/tree.h>
#include <libxml/xmlstring.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline xsltStylesheetPtr
xsltNewStylesheet(void)
{
	return nullptr;
}

static inline xsltStylesheetPtr
xsltParseStylesheetDoc(xmlDocPtr doc)
{
	(void)doc;
	return nullptr;
}

static inline xsltStylesheetPtr
xsltParseStylesheetFile(const xmlChar* filename)
{
	(void)filename;
	return nullptr;
}

static inline xmlDocPtr
xsltApplyStylesheet(xsltStylesheetPtr style, xmlDocPtr doc, const char** params)
{
	(void)style;
	(void)params;
	return doc;
}

static inline void
xsltFreeStylesheet(xsltStylesheetPtr style)
{
	(void)style;
}

static inline void
xsltCleanupGlobals(void)
{
}

#ifdef __cplusplus
}
#endif

#endif // LIBXSLT_TRANSFORM_H
