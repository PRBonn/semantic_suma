#ifndef RANGEPARAMETER_H_
#define RANGEPARAMETER_H_

#include "Parameter.h"
#include "CompositeParameter.h"
#include <iterator>
#include <list>

namespace rv
{

/*****************************************************************************\
 rose

 \headerfile RangeParameter.h
 \brief A RangeParameter is defined by arbitrary params, which are incrementable by "+=" and have an ordering by "<="

 For a RangeParameter you can use arbitrary parameters, as long as they support
 incrementation by += and have an ordering using the less operator.
 For instance, the following begin, end, increment values would result in an
 Range::const_iterator which produces the values 0.0, 0.2, 0.4:

 FloatParameter begin("begin", 0.0);
 FloatParameter end("end", 0.4);
 FloatParameter increment("increment", 0.4)

 RangeParameter("range", begin, end, increment);

 However, the same result is possible when taking a value f, which is in between 0.4 and 0.6, i.e.,

 FloatParameter end("end", 0.5);

 would yield the same result, when using a const_iterator.
 (This is why we need the less operator, i.e., operator<=. ;))


 \author behley

 \*****************************************************************************/
class RangeParameter: public CompositeParameter
{
	public:
		class ConstIterator
		{
				friend class RangeParameter;
			public:
				typedef Parameter value_type;
				typedef Parameter* pointer;
				typedef Parameter& reference;
				typedef const Parameter& const_reference;
				typedef size_t size_type;
				typedef ptrdiff_t difference_type;
				typedef std::input_iterator_tag iterator_category;

				ConstIterator(const ConstIterator& other) :
					mCurrentValue(other.mCurrentValue)
				{

				}

				ConstIterator& operator=(const ConstIterator& other)
				{
					if (&other == this) return *this;

					mCurrentValue = other.mCurrentValue;

					return *this;
				}

				const_reference operator*() const
				{
					return **mCurrentValue;
				}

				pointer operator->() const
				{
					return *mCurrentValue;
				}
				bool operator ==(const ConstIterator& other) const
				{
					return mCurrentValue == other.mCurrentValue;
				}

				bool operator !=(const ConstIterator& other) const
				{
					return mCurrentValue != other.mCurrentValue;
				}

				ConstIterator& operator++()
				{
					++mCurrentValue;
					return *this;
				}

				ConstIterator operator++(int)
				{
					ConstIterator it = *this;

					++mCurrentValue;

					return it;
				}
			protected:
				ConstIterator(const std::list<Parameter*>::const_iterator& start)
				: mCurrentValue(start)
				{

				}

				std::list<Parameter*>::const_iterator mCurrentValue;
		};

		typedef ConstIterator const_iterator;

		RangeParameter(const std::string& name, const Parameter& begin,
				const Parameter& end, const Parameter& increment, const std::string& operation);
		RangeParameter(const std::string& name, const ParameterList& values);
		RangeParameter(const std::string& name, const XmlNode& node);

		RangeParameter(const RangeParameter& other);
		~RangeParameter();

		RangeParameter& operator=(const RangeParameter& other);
		virtual RangeParameter* clone() const;

		std::string toString() const;

		std::string type() const
		{
			return "range";
		}

		const_iterator begin() const
		{
			// rename the parameter.
			return const_iterator(values.begin());
		}

		const_iterator end() const
		{
			return const_iterator(values.end());
		}

	protected:
		std::list<Parameter*> values;
};

RangeParameter* parseRangeParameter(const XmlNode& node);

}
#endif /* RANGEPARAMETER_H_ */
