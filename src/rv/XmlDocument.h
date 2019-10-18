#ifndef CORE_XMLDOCUMENT_H_
#define CORE_XMLDOCUMENT_H_

#include "XmlNode.h"
#include "XmlError.h"

/** \brief 
 *  
 *  \author behley
 */

namespace rv
{

class XmlDocument
{
  public:
    /** \brief Returns the root node of the XML document. */
    const XmlNode& root() const;

    /** \brief Reads an XML document from a file.
     *
     *  \param[in] fileName the file to be read
     *  \param[in] partial    do we generate document for partial/incomplete XML files? (default: false)
     *  \throw XmlError if the file could not be read or is malformed
     */
    static XmlDocument fromFile(const std::string& fileName, bool partial = false);

    /** \brief Reads an XML document from a string.
     *  \param[in] buffer
     *  \param[in] partial    do we generate document for partial/incomplete XML files? (default: false)
     *  \throw XmlError if the document is malformed
     */
    static XmlDocument fromData(const std::string& buffer, bool partial = false);

    /** \brief Reads an XML document from a input stream.
     *
     * \param in        input stream
     * \param partial   do we generate document for partial/incomplete XML files? (default: false)
     * \return xml document
     * \throw XmlError if the document is malformed
     */
    static XmlDocument fromStream(std::istream& in, bool partial = false);

  protected:
    XmlDocument();

    XmlNode root_;
};

} /* namespace rv */
#endif /* SRC_CORE_RV_XMLDOCUMENT_H_ */
