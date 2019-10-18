#include "rv/ParameterListIterator.h"
#include <cassert>

namespace rv
{

ParameterListIterator::ParameterListIterator(const ParameterList& params) :
  mHasNext(true), mOriginalParams(params)
{
  parseParameterList(mCurrentParams, mOriginalParams);
  /** important: get iterators, when mRanges is initialized **/
  mRangeIters.reserve(mRangeParams.size());
  for (uint32_t i = 0; i < mRangeParams.size(); ++i)
    mRangeIters.push_back(mRangeParams[i].begin());
}

ParameterListIterator::ParameterListIterator(const ParameterList& params,
    const std::vector<std::string>& names) :
  mHasNext(true), mOriginalParams(params), mFilter(names)
{
  parseParameterList(mCurrentParams, mOriginalParams);
  /** important: get iterators, when mRanges is initialized **/
  mRangeIters.reserve(mRangeParams.size());
  for (uint32_t i = 0; i < mRangeParams.size(); ++i)
    mRangeIters.push_back(mRangeParams[i].begin());
}

ParameterListIterator::ParameterListIterator(const ParameterListIterator& )
{
  /**
   * The problem with the current implementation is the RangeParameter.
   * Here we need to distinguish between range-values having the same value.
   * (e.g. <param name="value" type="integer">3</param> and <param name="another-value" type="integer">3</param>)
   *
   * with the current implementation of the ConstIterator, there is strict-aliasing problem...
   */
}

ParameterListIterator& ParameterListIterator::operator=(
    const ParameterListIterator& )
{
  return *this;
}

const std::vector<RangeParameter>& ParameterListIterator::getRangeParams() const
{
  return mRangeParams;
}

void ParameterListIterator::parseParameterList(ParameterList& new_params,
    const ParameterList& ranged_params)
{
  /** extract all range parameters and initialize currentParams. **/
  for (ParameterList::const_iterator it = ranged_params.begin(); it
      != ranged_params.end(); ++it)
  {
    const Parameter& param = *it;
    if (param.type() == "range")
    {
      const RangeParameter& rparam =
          dynamic_cast<const RangeParameter&> (param);

      bool found = false;
      for (uint32_t i = 0; i < mFilter.size(); ++i)
      {
        if (mFilter[i] == rparam.name())
        {
          found = true;
          break;
        }
      }

      if (found || mFilter.size() == 0)
      {
        mRangeParams.push_back(rparam);
        mRangeParamParents.push_back(&new_params);
      }
      else
      {
        /** insert range parameter **/
        new_params.insert(param);
      }
    }
    else if (param.type() == "composite")
    {
      const CompositeParameter& cparam =
          dynamic_cast<const CompositeParameter&> (param);
      new_params.insert(cparam);
      CompositeParameter* new_cparam = new_params.getParameter<
          CompositeParameter> (cparam.name());
      /** parse composite and instantiate range params **/
      parseParameterList(new_cparam->getParams(), cparam.getParams());
    }
    else
      new_params.insert(param);
  }
}

bool ParameterListIterator::hasNext()
{
  return mHasNext;
}

const ParameterList& ParameterListIterator::next()
{
  assert(mHasNext);

  /** replace RangeParameters in currentParams **/
  for (uint32_t i = 0; i < mRangeParams.size(); ++i)
  {
    mRangeParamParents[i]->insert(*mRangeIters[i]);
  }

  /** increment RangeParameters and set hasNext **/
  uint32_t i = 0;
  for (; i < mRangeParams.size(); ++i)
  {
    ++mRangeIters[i];
    if (mRangeIters[i] == mRangeParams[i].end())
    {
      mRangeIters[i] = mRangeParams[i].begin();
      continue; /** try to increment next iterator **/
    }
    break; /** leave loop **/
  }

  /** if we incremented all iterators till the end, we are finished. **/
  mHasNext = (i != mRangeParams.size());

  return mCurrentParams;
}

}
