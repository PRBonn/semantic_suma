#ifndef INCLUDE_CORE_MESH_H_
#define INCLUDE_CORE_MESH_H_

#include <glow/GlBuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlVertexArray.h>
#include <glow/glutil.h>


class Model;

/** \brief Representation of a mesh with possibly multiple materials.
 *
 *  A mesh contains all vertex buffers for drawing.
 *
 *  \author behley
 */
class Mesh {
  friend class Model;

 public:
  struct Material {
   public:
    Material() : ambient(0.1, 0.1, 0.1), diffuse(0.0, 0.0, 0.9), specular(0, 0, 0), emission(0, 0, 0) {}
    glow::vec3 ambient;
    glow::vec3 diffuse;
    glow::vec3 specular;
    glow::vec3 emission;

    float shininess{1.0f};  // specular exponent
    float alpha{1.0f};
  };

  struct Vertex {
   public:
    glow::vec4 position;
    glow::vec4 normal;
    glow::vec2 texture;
  };

  struct Triangle {
   public:
    uint32_t vertices[3];
    uint32_t material{0};
  };

  Mesh();
  Mesh(const std::vector<Vertex>& vertices, const std::vector<Triangle>& faces);
  Mesh(const std::vector<Vertex>& vertices, const std::vector<Triangle>& faces, const std::vector<Material>& materials);

  void draw(glow::GlProgram& program) const;

  const Material& material(uint32_t idx) const {
    if (idx >= materials_.size()) throw std::runtime_error("non-existent material.");
    return materials_[idx].value();
  }

 protected:
  void initialize(const std::vector<Vertex>& vertices, const std::vector<Triangle>& faces,
                  const std::vector<Material>& materials);

  float length(const glow::vec4& v);

  glow::GlBuffer<Vertex> vertices_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STATIC_DRAW};
  std::vector<glow::GlVertexArray> vaos_;
  std::vector<glow::GlBuffer<uint32_t> > triangles_;  // per material triangles.
  std::vector<glow::GlUniform<Material> > materials_;

  //    std::vector<Vertex> vertices_;
  //    std::vector<std::vector<uint32_t> > ;
};

#endif /* INCLUDE_CORE_MESH_H_ */
