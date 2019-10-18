#ifndef CORE_XMLNODE_H_
#define CORE_XMLNODE_H_

#include <string>
#include <vector>
#include <map>
#include <stdint.h>

/** \brief XmlNode in the DOM representation of an XmlDocument.
 * 
 *
 * Disclaimer: class is inspired by RoSe::XmlNode by Timo Röhling.
 *
 * \author behley
 */

namespace rv
{

class XmlNode
{
  public:
    friend class XmlDocument;

    XmlNode();
//    ~XmlNode();
//    XmlNode(const XmlNode& other);
//    XmlNode& operator=(const XmlNode& other);

    /** \brief List of nodes. */
    typedef std::vector<XmlNode> List;

    /** \brief Is XmlNode valid? **/
    bool isValid() const;

    /** \brief Returns the tag name. */
    const std::string& getName() const;

    /** \brief Returns the node content, i.e. the data between opening and closing tag */
    const std::string& getContent() const;

    uint32_t getLineNo() const;

    /** \brief Returns true if the node has at least one child node with the given tag name. */
    bool hasChild(const std::string& name) const;

    /** \brief Returns true if the node has the given attribute. */
    bool hasAttribute(const std::string& name) const;

    /** \brief Returns the value of the given attribute.
     * \throw XmlError if the node has no attribute \a name */
    std::string getAttribute(const std::string& name) const;

    /** \brief Returns a list of all child nodes. */
    const List& getChildren() const;

    /** \brief Returns a list of all child nodes with the given tag name. */
    List getChildren(const std::string& name) const;
    /** \brief Returns the first child node with the given tag name.
     *
     * If no such node exists, an invalid node is returned.
     */
    XmlNode getChild(const std::string& name) const;
    /** \brief Returns a list of all child nodes which have the given attribute and value. */

    List getChildrenWith(const std::string& attribute, const std::string& value) const;
    /** \brief Returns the first child node that has the given attribute.
     *
     * If no such node exists, an invalid node is returned.
     */
    XmlNode getChildWith(const std::string& name, const std::string& value) const;

  protected:
    bool valid_;
    std::string tag_name_;
    std::string content_;
    std::map<std::string, std::string> attributes_;
    uint32_t lineNumber_;

    std::vector<XmlNode> children_;
};

} /* namespace rv */
#endif /* CORE_XMLNODE_H_ */
