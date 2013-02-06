
#ifndef _OBJECT_MANIPULATOR_CONVERT_FUNCTIONS_H_
#define _OBJECT_MANIPULATOR_CONVERT_FUNCTIONS_H_

#include <tf/tf.h>


namespace object_manipulator
{

namespace convert_functions
{

  inline tf::Matrix3x3 createMatrix(const tf::Vector3 &X, const tf::Vector3 &Y, const tf::Vector3 &Z)
  {
    return tf::Matrix3x3( X.x(), Y.x(), Z.x(),
                        X.y(), Y.y(), Z.y(),
                        X.z(), Y.z(), Z.z() );

  }

  inline void setMatrix(tf::Matrix3x3 &mat, const tf::Vector3 &X, const tf::Vector3 &Y, const tf::Vector3 &Z)
  {
    mat.setValue( X.x(), Y.x(), Z.x(),
                  X.y(), Y.y(), Z.y(),
                  X.z(), Y.z(), Z.z() );

  }



} // namespace convert_functions

} // namespace object_manipulator

#endif



