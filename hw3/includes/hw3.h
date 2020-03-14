#ifndef __HW2_H__
#define __HW2_H__

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/string_cast.hpp"
#include "vertex.h"
#include "camera.h"

#include <cmath>
#include <random>

using std::string;

void loadShader(const char *, const char *, unsigned int &, unsigned int &,
                unsigned int &);


#endif
