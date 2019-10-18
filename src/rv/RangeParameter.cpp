#include "rv/RangeParameter.h"
#include <sstream>
#include <cassert>

namespace rv
{

RangeParameter::RangeParameter(const std::string& name, const Parameter& begin,
		const Parameter& end, const Parameter& increment,
		const std::string& ) :
	CompositeParameter(name)
{
	Parameter* param = begin.clone();
	param->name() = name;
	Parameter* endRenamed = end.clone();
	endRenamed->name() = name;

	for (; *param <= *endRenamed; *param += increment)
		values.push_back(param->clone());

//	values.push_back(endRenamed->clone()); // needed to ensure <=

	delete endRenamed;
	delete param;

	param_list.insert(begin);
	param_list.insert(end);
	param_list.insert(increment);
}

RangeParameter::RangeParameter(const std::string& name,
		const XmlNode& node) :
	CompositeParameter(name, node)
{
	// determine if for or sequence
	if (param_list.hasParam("begin") && param_list.hasParam("end")
			&& param_list.hasParam("increment"))
	{
		Parameter* param = param_list["begin"].clone();
		param->name() = name;
		Parameter* endRenamed = param_list["end"].clone();
		endRenamed->name() = name;

		for (; *param <= *endRenamed; *param += param_list["increment"])
			values.push_back(param->clone());

//		values.push_back(endRenamed->clone());
		delete endRenamed;
		delete param;
	}
	else
	{
		for (ParameterList::const_iterator it = param_list.begin(); it
				!= param_list.end(); ++it)
		{
			Parameter* param = it->clone();
			// rename this.
			param->name() = name;
			values.push_back(param);
		}
	}

	if(values.begin() == values.end())
		throw Exception("RangeParameter initialization failed! No child params found.");

}

RangeParameter::RangeParameter(const std::string& name, const ParameterList& vals)
: CompositeParameter(name, vals)
{
	for (ParameterList::const_iterator it = param_list.begin(); it
			!= param_list.end(); ++it)
	{
		Parameter* param = it->clone();
		// rename this.
		param->name() = name;
		values.push_back(param);
	}
}

RangeParameter::~RangeParameter()
{
	for (std::list<Parameter*>::const_iterator it = values.begin(); it
			!= values.end(); ++it)
		delete *it;
	values.clear();
}

RangeParameter::RangeParameter(const RangeParameter& other)
: CompositeParameter(other)
{
	for (std::list<Parameter*>::const_iterator it = other.values.begin(); it
			!= other.values.end(); ++it)
		values.push_back((*it)->clone());
}

RangeParameter& RangeParameter::operator=(const RangeParameter& other)
{
	if (&other == this) return *this;

	for (std::list<Parameter*>::const_iterator it = values.begin(); it
			!= values.end(); ++it)
		delete *it;
	values.clear();

	for (std::list<Parameter*>::const_iterator it = other.values.begin(); it
			!= other.values.end(); ++it)
		values.push_back((*it)->clone());

	return *this;
}

RangeParameter* RangeParameter::clone() const
{
	return new RangeParameter(*this);
}

std::string RangeParameter::toString() const
{
	std::stringstream out;

	out << "<param name=\"" << mName << "\" type=\"range\">" << std::endl;
	for (ParameterList::const_iterator it = param_list.begin(); it
			!= param_list.end(); ++it)
		out << "   " << *it << std::endl;
	out << "</param>";

	return out.str();
}

RangeParameter* parseRangeParameter(const XmlNode& node)
{
	if (node.getAttribute("type") == "range")
		return new RangeParameter(node.getAttribute("name"), node);
	else
		throw XmlError("Error while parameter parsing: 'type' not 'string'");
}

}
