/*****************************************************************************\
 \brief Parameter as a composition of other Parameters.

 \headerfile CompositeParameter.h
 \author behley

 \*****************************************************************************/

#ifndef COMPOSITEPARAMETER_H_
#define COMPOSITEPARAMETER_H_

#include "Parameter.h"
#include "ParameterList.h"

namespace rv
{

class CompositeParameter: public Parameter
{
	public:
		CompositeParameter(const std::string& name);
		CompositeParameter(const std::string& name, const XmlNode& value);
		CompositeParameter(const std::string& name, const ParameterList& values);

		CompositeParameter(const CompositeParameter& other);
		virtual CompositeParameter& operator=(const CompositeParameter& other);

		virtual ~CompositeParameter();

		virtual CompositeParameter* clone() const;

		virtual operator ParameterList() const;

		const ParameterList& getParams() const
		{
			return param_list;
		}

		ParameterList& getParams()
		{
		  return param_list;
		}

		virtual std::string toString() const;

		virtual inline XmlNode& value()
		{
			return val;
		}

		std::string type() const
		{
			return "composite";
		}

		virtual bool operator==(const Parameter& other) const
		{
			const CompositeParameter* param = dynamic_cast<const CompositeParameter*>(&other);
			if(param == 0) return false;

			return (other.name() == mName && param_list == param->param_list);
		}

	protected:
		ParameterList param_list;
		XmlNode val;
};

CompositeParameter* parseCompositeParameter(const XmlNode& node);

}
#endif /* COMPOSITEPARAMETER_H_ */
