#include "rv/XmlNode.h"
#include "rv/XmlError.h"

namespace rv
{

XmlNode::XmlNode() :
    valid_(false), lineNumber_(0)
{

}

//XmlNode::~XmlNode()
//{
//  for (uint32_t i = 0; i < children_.size(); ++i)
//    delete children_[i];
//}

//XmlNode::XmlNode(const XmlNode& other) :
//    valid_(other.valid_), tag_name_(other.tag_name_), content_(other.content_), attributes_(other.attributes_), lineNumber_(
//        other.lineNumber_)
//{
//  for (uint32_t i = 0; i < other.children_.size(); ++i)
//    children_.push_back(new XmlNode(*other.children_[i]));
//}

//XmlNode& XmlNode::operator=(const XmlNode& other)
//{
//  valid_ = other.valid_;
//  tag_name_ = other.tag_name_;
//  content_ = other.content_;
//  attributes_ = other.attributes_;
//  lineNumber_ = other.lineNumber_;
//
//  for (uint32_t i = 0; i < children_.size(); ++i)
//    delete children_[i];
//  children_.clear();
//
//  for (uint32_t i = 0; i < other.children_.size(); ++i)
//    children_.push_back(new XmlNode(*other.children_[i]));
//
//  return *this;
//}

bool XmlNode::isValid() const
{
  return valid_;
}

/** \brief Returns the tag name. */
const std::string& XmlNode::getName() const
{
  return tag_name_;
}

/** \brief Returns the node content, i.e. the data between opening and closing tag */
const std::string& XmlNode::getContent() const
{
  return content_;
}

uint32_t XmlNode::getLineNo() const
{

  return lineNumber_;
}

/** \brief Returns true if the node has at least one child node with the given tag name. */
bool XmlNode::hasChild(const std::string& name) const
{
  for (List::const_iterator it = children_.begin(); it != children_.end(); ++it)
    if (it->tag_name_ == name) return true;

  return false;
}

/** \brief Returns true if the node has the given attribute. */
bool XmlNode::hasAttribute(const std::string& name) const
{
  return (attributes_.find(name) != attributes_.end());
}

/** \brief Returns the value of the given attribute.
 * \throw XmlError if the node has no attribute \a name */
std::string XmlNode::getAttribute(const std::string& name) const
{
  if (attributes_.find(name) == attributes_.end()) throw XmlError("Attribute not found");

  return attributes_.find(name)->second;
}

/** \brief Returns a list of all child nodes. */
const XmlNode::List& XmlNode::getChildren() const
{
  return children_;
}

XmlNode::List XmlNode::getChildren(const std::string& name) const
{
  XmlNode::List matches;
  for (List::const_iterator it = children_.begin(); it != children_.end(); ++it)
    if (it->tag_name_ == name) matches.push_back(*it);

  return matches;
}

XmlNode XmlNode::getChild(const std::string& name) const
{
  for (List::const_iterator it = children_.begin(); it != children_.end(); ++it)
  {
    if (it->tag_name_ == name) return *it;
  }

  return XmlNode();
}

XmlNode::List XmlNode::getChildrenWith(const std::string& name, const std::string& value) const
{
  XmlNode::List matches;
  for (List::const_iterator it = children_.begin(); it != children_.end(); ++it)
  {
    if (!it->hasAttribute(name)) continue;
    if (it->attributes_.find(name)->second == value) matches.push_back(*it);
  }

  return matches;
}

XmlNode XmlNode::getChildWith(const std::string& name, const std::string& value) const
{
  for (List::const_iterator it = children_.begin(); it != children_.end(); ++it)
  {
    if (!it->hasAttribute(name)) continue;
    if (it->attributes_.find(name)->second == value) return *it;
  }

  return XmlNode();
}

} /* namespace rv */

