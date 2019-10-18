#ifndef CORE_PRIMITIVEPARAMETERS_H_
#define CORE_PRIMITIVEPARAMETERS_H_

#include <string>
#include <sstream>

#include "Parameter.h"
#include "Exception.h"
#include <cmath>

namespace rv
{

/**
 * \brief Parameter representing a boolean value.
 *
 * \author behley
 */
class BooleanParameter: public Parameter
{
  public:
    BooleanParameter(const std::string& name, const bool value);

    operator bool() const
    {
      return val;
    }

    BooleanParameter* clone() const;

    std::string toString() const;
    std::string valueStr() const
    {
      std::stringstream sstr;
      sstr << (val?"true":"false");
      return sstr.str(); /** empty string if not appropriately implemented. **/
    }

    inline bool& value()
    {
      return val;
    }

    inline bool value() const
    {
      return val;
    }

    std::string type() const
    {
      return "boolean";
    }

    bool operator==(const Parameter& other) const
    {
      const BooleanParameter* param =
          dynamic_cast<const BooleanParameter*> (&other);
      if (param == 0) return false;

      return (other.name() == mName && param->val == val);
    }
  protected:
    bool val;
};

/**
 * \brief Parameter representing a integer value.
 *
 * \author behley
 */
class IntegerParameter: public Parameter
{
  public:
    IntegerParameter(const std::string& name, const int value);

    operator int() const
    {
      return val;
    }

    operator unsigned int() const
    {
      return val;
    }

    IntegerParameter* clone() const;

    std::string toString() const;
    std::string valueStr() const
    {
      std::stringstream sstr;
      sstr << val;
      return sstr.str();
    }

    inline int& value()
    {
      return val;
    }

    inline int value() const
    {
      return val;
    }

    std::string type() const
    {
      return "integer";
    }

    bool operator==(const Parameter& other) const
    {
      const IntegerParameter* param =
          dynamic_cast<const IntegerParameter*> (&other);
      if (param == 0) return false;

      return (other.name() == mName && param->val == val);
    }

    Parameter& operator+=(const Parameter& rhs)
    {
      const IntegerParameter* param =
          dynamic_cast<const IntegerParameter*> (&rhs);
      if (param == 0)
      {
        std::stringstream str;
        str << "operator+= not defined for parameter of type '" << rhs.type()
            << "'";
        throw Exception(str.str());
      }

      val += param->value();

      return *this;
    }

    bool operator<=(const Parameter& rhs)
    {
      const IntegerParameter* param =
          dynamic_cast<const IntegerParameter*> (&rhs);
      if (param == 0)
      {
        std::stringstream str;
        str << "operator< not defined for parameter of type '" << rhs.type()
            << "'";
        throw Exception(str.str());
      }

      return val <= param->value();
    }

  protected:
    int val;
};

class FloatParameter: public Parameter
{
  public:
    FloatParameter(const std::string& name, const double value);

    FloatParameter* clone() const;

    std::string toString() const;
    std::string valueStr() const
    {
      std::stringstream sstr;
      sstr << val;
      return sstr.str(); /** empty string if not appropriately implemented. **/
    }

    operator float() const
    {
      return val;
    }

    operator double() const
    {
      return val;
    }

    inline double& value()
    {
      return val;
    }

    inline double value() const
    {
      return val;
    }

    std::string type() const
    {
      return "float";
    }

    bool operator==(const Parameter& other) const
    {
      const FloatParameter* param =
          dynamic_cast<const FloatParameter*> (&other);
      if (param == 0) return false;

      // epsilon test.
      return (other.name() == mName && (std::abs(val - param->val)
          < 0.000000001));
    }

    Parameter& operator+=(const Parameter& rhs)
    {
      const FloatParameter* param = dynamic_cast<const FloatParameter*> (&rhs);
      if (param == 0)
      {
        std::stringstream str;
        str << "operator+= not defined for parameter of type '" << rhs.type()
            << "'";
        throw Exception(str.str());
      }

      val += param->value();

      return *this;
    }

    bool operator<=(const Parameter& rhs)
    {
      const FloatParameter* param = dynamic_cast<const FloatParameter*> (&rhs);
      if (param == 0)
      {
        std::stringstream str;
        str << "operator< not defined for parameter of type '" << rhs.type()
            << "'";
        throw Exception(str.str());
      }

      return val <= param->value();
    }

  protected:
    double val;
};

class StringParameter: public Parameter
{
  public:
    StringParameter(const std::string& name, const std::string value);

    StringParameter* clone() const;

    std::string toString() const;
    std::string valueStr() const
    {
      std::stringstream sstr;
      sstr << val;
      return sstr.str(); /** empty string if not appropriately implemented. **/
    }

    operator std::string() const
    {
      return val;
    }

    bool operator==(const Parameter& other) const
    {
      const StringParameter* param =
          dynamic_cast<const StringParameter*> (&other);
      if (param == 0) return false;

      return (other.name() == mName && param->val == val);
    }

    inline std::string& value()
    {
      return val;
    }

    inline std::string value() const
    {
      return val;
    }

    std::string type() const
    {
      return "string";
    }

  protected:
    std::string val;
};

BooleanParameter* parseBooleanParameter(const XmlNode& node);
IntegerParameter* parseIntegerParameter(const XmlNode& node);
FloatParameter* parseFloatParameter(const XmlNode& node);
StringParameter* parseStringParameter(const XmlNode& node);

}
#endif /* PRIMITIVEPARAMETERS_H_ */
