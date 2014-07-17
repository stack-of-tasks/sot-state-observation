#ifndef SOTSTATEOBSERVATIONTOOLSDEFINITIONS
#define SOTSTATEOBSERVATIONTOOLSDEFINITIONS

#include <state-observation/tools/definitions.hpp>
#include <dynamic-graph/linear-algebra.h>
#include <boost/type_traits/is_same.hpp>

// basic file operations
#include <iostream>
#include <fstream>
using namespace std;


namespace sotStateObservation
{

    template <typename T2V,typename T1V>
    inline T2V convertVector(const T1V & v0)
    {
        T2V v;
        v.resize(v0.size());
        for (unsigned i=0;i<v0.size();++i)
        {
            v(i)=v0(i);
        }
        return v;
    }

    template <typename T1>
    inline T1 convertVector(const T1 & v0)
    {
        return v0;
    }

    template <typename T2V,typename T1V>
    inline T2V convertMatrix(const T1V & v0)
    {
        T2V v;
        v.resize(v0.rows(),v0.cols());
        for (unsigned i=0;i<v0.rows();++i)
        {
            for (unsigned j=0;j<v0.cols();++j)
            {
                v(i,j)=v0(i,j);
            }

        }
        return v;
    }

    template <typename T2V>
    inline T2V convertMatrix(const dynamicgraph::Matrix & v0)
    {
        T2V v;
        v.resize(v0.nbRows(),v0.nbCols());
        for (unsigned i=0;i<v0.nbRows();++i)
        {
            for (unsigned j=0;j<v0.nbCols();++j)
            {
                v(i,j)=v0(i,j);
            }

        }
        return v;
    }

    template <typename T1>
    inline T1 convertMatrix(const T1 & v0)
    {
        return v0;
    }

    template<>
    inline dynamicgraph::Matrix convertMatrix
                    <dynamicgraph::Matrix, dynamicgraph::Matrix>
                                            (const dynamicgraph::Matrix & v0)
    {
        return v0;
    }

    template <typename Vector_t>
    inline Vector_t getSubvector(const Vector_t &, unsigned , unsigned )
    {
        BOOST_STATIC_ASSERT((boost::is_same<Vector_t, maal::boost::Vector>::value
                            ||boost::is_same<Vector_t, Eigen::VectorXd>::value));
    }

    template <>
    inline maal::boost::Vector getSubvector
        (const maal::boost::Vector& v, unsigned begin, unsigned length)
    {
        BOOST_ASSERT(v.size()>=(begin+length) && "The subvector is incorrectly set.");

        maal::boost::Vector v2(length);

        for (unsigned i=0 ; i<length ; ++i)
        {
            v2 (i)= v(i+begin);
        }
        return v2;

    }

    template <>
    inline Eigen::VectorXd getSubvector
        (const Eigen::VectorXd & v, unsigned begin, unsigned length)
    {
        return v.segment(begin,length);
    }

    template <typename Vector_t>
    inline Vector_t setSubvector(Vector_t &, unsigned , const Vector_t & )
    {
        BOOST_STATIC_ASSERT((boost::is_same<Vector_t, maal::boost::Vector>::value
                            ||boost::is_same<Vector_t, Eigen::VectorXd>::value));
    }

    template <>
    inline maal::boost::Vector setSubvector
        (maal::boost::Vector& v, unsigned begin, const maal::boost::Vector &vin)
    {

        BOOST_ASSERT (v.size()>=(begin+vin.size())
                             && "The subvector is incorrectly set.");

        unsigned l = vin.size();
        for (unsigned i=0 ; i < l ; ++i)
        {
            v (i+begin)= vin(i);
        }
        return v;
    }

    template <>
    inline Eigen::VectorXd setSubvector
        (Eigen::VectorXd & v, unsigned begin, const Eigen::VectorXd &vin)
    {

        BOOST_ASSERT (v.size()>=(begin+vin.size())
                            && "The subvector is incorrectly set.");

        v.segment(begin,v.size())=vin;
        return v;
    }

    template <typename Vector_t>
    inline Vector_t crossProduct(const Vector_t & v1, const Vector_t & v2 )
    {
        return v1^v2;
    }

    template<>
    inline Eigen::VectorXd crossProduct
                (const Eigen::VectorXd & v1, const Eigen::VectorXd & v2)
    {
        return Eigen::Vector3d(v1).cross(Eigen::Vector3d(v2));
    }

    template<>
    inline maal::boost::Vector crossProduct
                (const maal::boost::Vector & v1, const maal::boost::Vector& v2)
    {
        maal::boost::Vector v(3);

        v(0) = v1(1) * v2(2) - v1(2) * v2(1) ;
        v(1) = v1(2) * v2(0) - v1(0) * v2(2) ;
        v(2) = v1(0) * v2(1) - v1(1) * v2(0) ;

        return v;
    }



}

#endif // SOTSTATEOBSERVATIONTOOLSDEFINITIONS
