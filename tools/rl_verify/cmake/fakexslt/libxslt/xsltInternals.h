//
// Fake libxslt/xsltInternals.h — minimal substitute so that RL 0.7.0's
// rl::xml::Stylesheet.h compiles without a real libxslt installation.
//
// The only RL core user of libxslt is rl::xml::Stylesheet (XSLT transforms),
// which neither rl::mdl::XmlFactory nor any of our code exercises at runtime
// (the Comau rlmdl is not an XSLT stylesheet). rlmdl/rlkin merely compile the
// class, so we provide the xslt types and no-op function definitions.
//
// Never mix this header with a real libxslt; the functions are defined
// static-inline and would silently shadow real ones.
//

#ifndef LIBXSLT_XSLTINTERNALS_H
#define LIBXSLT_XSLTINTERNALS_H

#include <libxml/tree.h>
#include <libxml/xmlstring.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _xsltStylesheet
{
	xmlDocPtr doc;
};

typedef struct _xsltStylesheet xsltStylesheet;
typedef xsltStylesheet* xsltStylesheetPtr;

struct _xsltTransformContext
{
	int dummy;
};

typedef struct _xsltTransformContext xsltTransformContext;
typedef xsltTransformContext* xsltTransformContextPtr;

#ifdef __cplusplus
}
#endif

#endif // LIBXSLT_XSLTINTERNALS_H
