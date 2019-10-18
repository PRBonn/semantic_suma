#ifndef PARAMETERLISTITERATOR_H_
#define PARAMETERLISTITERATOR_H_

#include "ParameterList.h"
#include "RangeParameter.h"
#include <vector>

namespace rv
{
/**
 * \brief iterates over a given ParameterList and replacing all RangeParameters with appropriate values.
 *
 * Generate instantiated ParameterList where all RangeParameters are replaced by appropriate values.
 * If no values are RangeParameters then only the ParameterList is returned, when next is called.
 *
 * \author: behley
 */
class ParameterListIterator
{
  public:
    ParameterListIterator(const ParameterList& params);

    /** \brief iterate only partially over some RangeParameters
     *
     * Only some of the RangeParameters are replaced, the rest are left in their original state. Thus,
     * it is possible to iterate in a nested fashion over the Parameters.
     */
    ParameterListIterator(const ParameterList& params,
        const std::vector<std::string>& names);

    /** \brief new ParameterList with instantiated range params available? **/
    bool hasNext();
    /** \brief next ParameterList with instantiated Parameters, and advance to next params **/
    const ParameterList& next();
    /** \brief returns the RangeParameter, which are replaced. **/
    const std::vector<RangeParameter>& getRangeParams() const;

  protected:
    /** todo: implement copy constructors. **/
    ParameterListIterator(const ParameterListIterator& other);
    ParameterListIterator& operator=(const ParameterListIterator& other);

    /** \brief extract range parameters from ParameterList, and set parents. **/
    void parseParameterList(ParameterList& new_params,
        const ParameterList& ranged_params);

    bool mHasNext;
    /** idea for copy/assignment: use original parameter list, and iterator. **/
    ParameterList mOriginalParams;
    std::vector<std::string> mFilter;

    /** fully instantiated ParameterList **/
    ParameterList mCurrentParams;
    /** original range parameters **/
    std::vector<RangeParameter> mRangeParams;
    /** extracted range parameters **/
    std::vector<RangeParameter::const_iterator> mRangeIters;
    /** pointers to ParameterLists where the i-th RangeParameter occurs. **/
    std::vector<ParameterList*> mRangeParamParents;
};

}
#endif /* PARAMETERLISTITERATOR_H_ */
