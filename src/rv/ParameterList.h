#ifndef CORE_PARAMETERLIST_H_
#define CORE_PARAMETERLIST_H_

#include <string>
#include <map>
#include <cstddef>

#include "Parameter.h"

namespace rv
{
/*****************************************************************************
 \brief Encapsulates Parameter objects and provides some convenience functions.

 The class handles the removal of pointers and clones parameters,
 when needed. All parameters are free'd when the live time
 of the ParameterList ends.


 Example usage:

 ParameterList params;
 params.insert(IntegerParameter("int1", 10)); // inserts an integer parameter named int1 with value 10
 // the Parameter is cloned and a copy is stored inside ParameterList
 params.insert(new FloatParameter("float1", 1.337)); // inserts a float parameter named "float1" with value 1.337
 // IMPORTANT: the pointer is stored, and free'd, if the ParameterList is free'd!

 params.insert(StringParameter("int1", "test")); // replaces the IntegerParameter with a StringParameter with the same name!

 float value = params["float1"];         // returns the Parameter with name "float1", which is then converted to a float implicitly. (See Parameter)
 int value1 = params.getValue<int>("float1"); // explicit conversion, when type does not match. (See Parameter for which conversions are possible.)

 // check if a parameter with name "notInList" is present.
 if(params.hasParam("notInList"))
 ...

 // iterating through parameter values can be done using the iterator
 for(ParameterList::const_iterator it = params.begin(); it != params.end(); ++it)
 {
 ...
 }

 If a Parameter with a given name is not in the ParameterList a RoSe::Exception is thrown.

 \headerfile ParameterList.h
 \author behley

 \*****************************************************************************/

class ParameterList
{
  public:
    class ConstIterator
    {
        friend class ParameterList;
      public:
        typedef Parameter value_type;
        typedef Parameter* pointer;
        typedef Parameter& reference;
        typedef const Parameter& const_reference;
        typedef size_t size_type;
        typedef ptrdiff_t difference_type;
        typedef std::input_iterator_tag iterator_category;

        ConstIterator()
        {
        }

        ~ConstIterator()
        {

        }

        const_reference operator*() const
        {
          return *(mIterator->second);
        }

        pointer operator->() const
        {
          return (mIterator->second);
        }

        bool operator ==(const ConstIterator& other) const
        {
          return mIterator == other.mIterator;
        }

        bool operator !=(const ConstIterator& other) const
        {
          return !(other == *this);
        }

        ConstIterator& operator++()
        {
          ++mIterator;

          return *this;
        }

        ConstIterator operator++(int)
        {
          ConstIterator it = *this;
          ++mIterator;
          return it;
        }
      protected:
        ConstIterator(
            const std::map<std::string, Parameter*>::const_iterator& it) :
          mIterator(it)
        {

        }
        std::map<std::string, Parameter*>::const_iterator mIterator;
    };

    typedef ConstIterator const_iterator;

    ParameterList();
    ~ParameterList();

    ParameterList(const ParameterList& other);
    ParameterList& operator=(const ParameterList& other);

    bool operator==(const ParameterList& other) const;
    bool operator!=(const ParameterList& other) const;

    /* \brief adds a parameter to the list and replaces an already existing parameter with the same name */
    void insert(const Parameter& param);
    /* \brief adds a parameter to the list and replaces an already existing parameter with the same name
     * Warning: the pointer is not copied. The point will be delete with the parameter list.
     **/
    void insert(Parameter* param);

    /* \brief removes the parameter with name 'param' */
    void erase(const std::string& name);
    /* \brief is parameter with given name in the parameter list? */
    bool hasParam(const std::string& name) const;

    //		Parameter& operator[](const std::string& name)
    //		{
    //			checkParam(name);
    //			return *(params[name]);
    //		}

    const Parameter& operator[](const std::string& name) const
    {
      checkParam(name);
      return *(params.find(name)->second);
    }

    /* \brief get the value of parameter with name 'name' and tries to convert it to the given type */
    template<class T>
    T getValue(const std::string& name) const
    {
      checkParam(name);
      return static_cast<T> (*params.find(name)->second);
    }

    /* \brief get the parameter with the given name */
    template<class T>
    const T* getParameter(const std::string& name) const
    {
      checkParam(name);
      T* ptr = dynamic_cast<T*> (params.find(name)->second);
      if (ptr == 0)
      {
        std::stringstream str;
        str << "Parameter with name '" << name << "' not of suitable type.";
        throw Exception(str.str());
      }

      return ptr;
    }

    template<class T>
    T* getParameter(const std::string& name)
    {
      checkParam(name);
      T* ptr = dynamic_cast<T*> (params[name]);
      if (ptr == 0)
      {
        std::stringstream str;
        str << "Parameter with name '" << name << "' not of suitable type.";
        throw Exception(str.str());
      }

      return ptr;
    }

    const_iterator begin() const;
    const_iterator end() const;

    inline size_t size() const
    {
      return params.size();
    }

    void clear();

  protected:
    void checkParam(const std::string& name) const;
    std::map<std::string, Parameter*> params;
};

/** \brief parsing of a XML-file
 *
 *  Reading the file and extracting all <param> nodes attached to the root node.
 *
 *  \param filename name of the input file
 *  \param params ParameterList, where we insert the parsed Parameters
 *
 *  \throws RoSe::XmlError, if a parsing error occurs.
 */
void parseXmlFile(const std::string& filename, ParameterList& params);
}
#endif /* PARAMETERLIST_H_ */
