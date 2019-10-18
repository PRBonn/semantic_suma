#include "ObjReader.h"

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>

using namespace glow;

std::string trim(const std::string& str, const std::string& whitespaces = " \0\t\n\r\x0B") {
  int32_t beg = 0;
  int32_t end = 0;

  /** find the beginning **/
  for (beg = 0; beg < (int32_t)str.size(); ++beg) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[beg] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  /** find the end **/
  for (end = int32_t(str.size()) - 1; end > beg; --end) {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i) {
      if (str[end] == whitespaces[i]) {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  return str.substr(beg, end - beg + 1);
}

std::vector<std::string> split(const std::string& line, const std::string& delim, bool skipEmpty = false) {
  std::vector<std::string> tokens;

  boost::char_separator<char> sep(delim.c_str(), "", (skipEmpty ? boost::drop_empty_tokens : boost::keep_empty_tokens));
  boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);

  for (auto it = tokenizer.begin(); it != tokenizer.end(); ++it) tokens.push_back(*it);

  return tokens;
}

std::string dirName(const std::string& path) {
  boost::filesystem::path p(path);

  return p.parent_path().string();
}

Mesh ObjReader::fromFile(const std::string& filename) {
  //  std::cout << "ObjReader: " << filename << std::flush;

  struct ObjFace {
    int32_t vertices[3];
    int32_t normals[3];
    int32_t texcoords[3];
    int32_t material;
  };

  std::vector<Mesh::Vertex> vertices;
  std::vector<Mesh::Triangle> triangles;
  std::vector<Mesh::Material> materials;

  std::ifstream in(filename.c_str());
  if (!in.is_open()) throw std::runtime_error("Failed to open obj-file.");
  std::vector<std::string> tokens;
  std::string line;

  std::vector<glow::vec4> obj_verts;
  std::vector<glow::vec4> obj_normals;
  std::vector<glow::vec2> obj_texcoords;
  std::vector<ObjFace> obj_faces;
  std::map<std::string, uint32_t> material_map;

  int32_t current_material = -1;
  // 1. read first all information.
  // 2. assembly then the vertices with help of vert_normal.
  // 3. construct mesh primitives.

  in.peek();
  while (!in.eof()) {
    std::getline(in, line);
    tokens = split(line, " ");
    in.peek();
    if (tokens.size() < 2) continue;
    if (tokens[0] == "#") continue;

    if (tokens[0] == "mtllib") {
      std::map<std::string, Mesh::Material> mats;
      std::string mtl_filename = dirName(filename);
      mtl_filename += "/" + tokens[1];
      //      std::cout << ", " << mtl_filename << std::flush;
      parseMaterials(mtl_filename, mats);
      for (auto it = mats.begin(); it != mats.end(); ++it) {
        material_map[it->first] = materials.size();
        materials.push_back(it->second);
      }
    } else if (tokens[0] == "v")  // vertices.
    {
      if (tokens.size() < 4) throw std::runtime_error("Failure parsing vertex: not enough tokens.");
      float x = boost::lexical_cast<float>(tokens[1]);
      float y = boost::lexical_cast<float>(tokens[2]);
      float z = boost::lexical_cast<float>(tokens[3]);

      // FIXME: interpret also w coordinate.

      obj_verts.push_back(glow::vec4(x, y, z, 1.0));
    } else if (tokens[0] == "vn")  // normals
    {
      if (tokens.size() < 4) throw std::runtime_error("Failure parsing normal: not enough tokens.");
      float x = boost::lexical_cast<float>(tokens[1]);
      float y = boost::lexical_cast<float>(tokens[2]);
      float z = boost::lexical_cast<float>(tokens[3]);

      obj_normals.push_back(glow::vec4(x, y, z, 0.0));

      // FIXME: interpret also w coordinate.
    } else if (tokens[0] == "usemtl")  // material
    {
      std::string name = trim(tokens[1]);
      if (material_map.find(name) == material_map.end()) throw std::runtime_error("Unknown material.");
      current_material = material_map[name];
    } else if (tokens[0] == "f") {
      if (tokens.size() < 4) throw std::runtime_error("Failure parsing face: not enough tokens.");
      std::vector<std::string> face_tokens;
      ObjFace face;
      face.normals[0] = -1;
      face.normals[1] = -1;
      face.normals[2] = -1;
      face.material = current_material;

      for (uint32_t j = 1; j < 4; ++j) {
        face_tokens = split(tokens[j], "/");
        if (face_tokens.size() == 0) {
          //          std::cout << stringify(tokens) << std::endl;
          std::cout << tokens[j] << std::endl;
          throw std::runtime_error("Failure parsing face: insufficient vertex params");
        }
        if (face_tokens.size() > 0)  // vertex
        {
          face.vertices[j - 1] = boost::lexical_cast<uint32_t>(face_tokens[0]) - 1;
        }

        if (face_tokens.size() > 1)  // texture
        {
          if (face_tokens[1].size() > 0)  // non-empty
          {
            face.texcoords[j - 1] = boost::lexical_cast<uint32_t>(face_tokens[1]) - 1;
          }
        }

        if (face_tokens.size() > 2)  // normal
        {
          if (face_tokens[2].size() > 0)  // non-empty
          {
            face.normals[j - 1] = boost::lexical_cast<uint32_t>(face_tokens[2]) - 1;
          }
        }
      }

      obj_faces.push_back(face);
    }

    in.peek();
  }
  in.close();

  // getting normal; vertices together (map v * (N+1) + n -> index, where N is normals size)
  std::map<uint32_t, uint32_t> vert_normal;
  // generate for all faces the corresponding vertices:
  for (uint32_t f = 0; f < obj_faces.size(); ++f) {
    const ObjFace& face = obj_faces[f];
    Mesh::Triangle triangle;
    if (face.material >= 0) triangle.material = face.material;
    for (uint32_t i = 0; i < 3; ++i) {
      uint32_t key = face.vertices[i] * (obj_normals.size() + 1) + (face.normals[i] + 1);
      if (vert_normal.find(key) == vert_normal.end()) {
        Mesh::Vertex vertex;
        vertex.position = obj_verts[face.vertices[i]];
        if (face.normals[i] >= 0) vertex.normal = obj_normals[face.normals[i]];
        // FIXME: texture coordinates.

        triangle.vertices[i] = vertices.size();
        vert_normal[key] = vertices.size();
        vertices.push_back(vertex);
      } else {
        triangle.vertices[i] = vert_normal[key];
      }
    }
    triangles.push_back(triangle);
  }

  //  std::cout << "(v: " << obj_verts.size();
  //  std::cout << ", n: " << obj_normals.size();
  //  std::cout << ", f: " << obj_faces.size();
  //  std::cout << ", m: " << materials.size();

  //  std::cout << " v:" << vertices.size() << ", t:" << triangles.size() << ", m: " << materials.size()
  //      << std::endl;
  //  std::cout << ")...finished" << std::endl;
  if (materials.size() == 0)
    return Mesh(vertices, triangles);
  else
    return Mesh(vertices, triangles, materials);
}

void ObjReader::parseMaterials(const std::string& filename, std::map<std::string, Mesh::Material>& materials) {
  std::ifstream in(filename.c_str());
  if (!in.is_open()) throw std::runtime_error("Failed to open material file.");
  std::vector<std::string> tokens;
  std::string line;

  Mesh::Material* material = nullptr;

  in.peek();
  while (!in.eof()) {
    std::getline(in, line);
    tokens = split(line, " ");
    if (tokens.size() < 2) continue;
    if (tokens[0] == "#") continue;

    if (tokens[0] == "newmtl") {
      std::string name = trim(tokens[1]);
      materials[name] = Mesh::Material();
      material = &materials[name];
    } else if (material != nullptr) {
      if (tokens[0] == "Ka") {
        if (tokens.size() < 4) throw std::runtime_error("Failure parsing material file.");
        float r = boost::lexical_cast<float>(tokens[1]);
        float g = boost::lexical_cast<float>(tokens[2]);
        float b = boost::lexical_cast<float>(tokens[3]);
        material->ambient = glow::vec3(r, g, b);
      } else if (tokens[0] == "Kd") {
        if (tokens.size() < 4) throw std::runtime_error("Failure parsing material file.");
        float r = boost::lexical_cast<float>(tokens[1]);
        float g = boost::lexical_cast<float>(tokens[2]);
        float b = boost::lexical_cast<float>(tokens[3]);
        material->diffuse = glow::vec3(r, g, b);
      } else if (tokens[0] == "Ks") {
        if (tokens.size() < 4) throw std::runtime_error("Failure parsing material file.");
        float r = boost::lexical_cast<float>(tokens[1]);
        float g = boost::lexical_cast<float>(tokens[2]);
        float b = boost::lexical_cast<float>(tokens[3]);
        material->specular = glow::vec3(r, g, b);
      } else if (tokens[0] == "Ke") {
        if (tokens.size() < 4) throw std::runtime_error("Failure parsing material file.");
        float r = boost::lexical_cast<float>(tokens[1]);
        float g = boost::lexical_cast<float>(tokens[2]);
        float b = boost::lexical_cast<float>(tokens[3]);
        material->emission = glow::vec3(r, g, b);
      } else if (tokens[0] == "Ns") {
        material->shininess = boost::lexical_cast<float>(tokens[1]);
      } else if (tokens[0] == "d") {
        material->alpha = boost::lexical_cast<float>(tokens[1]);
      }
    }

    in.peek();
  }

  in.close();
}
