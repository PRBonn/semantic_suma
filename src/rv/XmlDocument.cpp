#include "rv/XmlDocument.h"

#include <cassert>
#include <fstream>
#include <sstream>
#include <iostream>
#include "yxml.h"
#include "rv/XmlError.h"

namespace rv
{

XmlDocument::XmlDocument()
{

}

const XmlNode& XmlDocument::root() const
{
  return root_;
}

XmlDocument XmlDocument::fromFile(const std::string& filename, bool partial)
{
  std::ifstream in(filename.c_str());
  XmlDocument doc = fromStream(in, partial);
  in.close();

  return doc;
}

XmlDocument XmlDocument::fromData(const std::string& buffer, bool partial)
{
  std::istringstream in(buffer);

  return fromStream(in, partial);
}

XmlDocument XmlDocument::fromStream(std::istream& stream, bool partial)
{
  void* buf = malloc(4096);
  yxml_t x;
  yxml_init(&x, buf, 4096);

  XmlDocument doc;
  std::vector<XmlNode*> currentNodes; // stack of nodes that are currently initialized.

  yxml_ret_t ret;

  XmlNode* node = 0;
  std::string attr_value;
  std::string content;

  stream.peek();
  while (!stream.eof())
  {
    ret = yxml_parse(&x, stream.get());

    switch (ret)
    {
      case YXML_ELEMSTART:
        node = 0;
        if (currentNodes.empty())
        {
          node = &doc.root_;
        }
        else
        {
          // Note: this could alter the pointers in currentNodes;
          // However, the invariant is that parent.children are already initialized!
          XmlNode* parent = currentNodes.back();
          parent->children_.push_back(XmlNode());
          node = &parent->children_.back();
        }

        assert(node != 0);
        node->valid_ = partial;
        node->lineNumber_ = x.line;
        node->tag_name_ = x.elem;

        currentNodes.push_back(node);
        content.clear();
        break;

      case YXML_CONTENT:
        content += x.data;
        break;

      case YXML_ELEMEND:
        node = currentNodes.back();

        if (node->children_.size() == 0) node->content_ = content;

        node->valid_ = true;
        currentNodes.pop_back();

        break;

      case YXML_ATTRSTART:
        attr_value.clear();
        break;

      case YXML_ATTRVAL:
        attr_value += x.data;
        break;

      case YXML_ATTREND:
        node = currentNodes.back();
        node->attributes_[x.attr] = attr_value;
        break;

      case YXML_OK:
      case YXML_PISTART:
      case YXML_PICONTENT:
      case YXML_PIEND:
        // unhandled.
        break;

      case YXML_EEOF:
      case YXML_EREF:
      case YXML_ECLOSE:
      case YXML_ESTACK:
      case YXML_ESYN:
        free(buf);

        std::stringstream reason;
        reason << "Error while parsing in line " << x.line;
        throw XmlError(reason.str());
        break;
    }

    stream.peek();
  }

  free(buf);

  if (!partial && (currentNodes.size() > 0)) throw XmlError("Error while parsing: Unexpected EOF.");

  return doc;
}

} /* namespace rv */
