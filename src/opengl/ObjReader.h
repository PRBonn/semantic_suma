#ifndef INCLUDE_CORE_OBJREADER_H_
#define INCLUDE_CORE_OBJREADER_H_

#include "Mesh.h"

/** \brief reading and parsing of Wavefront .obj files.
 *
 *  \author behley
 */
class ObjReader {
 public:
  /** \brief read mesh from file with given filename. **/
  static Mesh fromFile(const std::string& filename);

 protected:
  static void parseMaterials(const std::string& filename, std::map<std::string, Mesh::Material>& materials);
};

#endif /* INCLUDE_CORE_OBJREADER_H_ */
