#include "rv/Parameter.h"
#include "rv/PrimitiveParameters.h"
#include "rv/RangeParameter.h"
#include "rv/CompositeParameter.h"

#include "rv/ParameterList.h"

namespace rv
{

Parameter::operator ParameterList() const
{
  std::stringstream str;
  str << "conversion of Parameter with name '" << mName
      << "' to ParameterList not possible.";
  throw Exception(str.str());
}

Parameter* Parameter::parseParameter(const XmlNode& node)
{
  const std::string param_name = node.getAttribute("name");
  const std::string param_type = node.getAttribute("type");

  if (param_type == "integer")
    return parseIntegerParameter(node);
  else if (param_type == "float")
    return parseFloatParameter(node);
  else if (param_type == "string")
    return parseStringParameter(node);
  else if (param_type == "boolean")
    return parseBooleanParameter(node);
//  else if (param_type == "vector")
//    return parseVectorParameter(node);
//  else if (param_type == "matrix")
//    return parseMatrixParameter(node);
  else if (param_type == "range")
    return parseRangeParameter(node);
  else if (param_type == "composite")
    return parseCompositeParameter(node);
  else
  {
    std::stringstream reason;
    reason << "Unknown parameter type in xml file at line " << node.getLineNo()
        << std::endl;
    throw XmlError(reason.str());
  }

}

}
