#ifndef SOTSTATEOBSERVATIONTOOLSDEFINITIONS
#define SOTSTATEOBSERVATIONTOOLSDEFINITIONS

#include <state-observation/tools/definitions.hpp>
#include <dynamic-graph/linear-algebra.h>
#include <boost/type_traits/is_same.hpp>


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
    inline Vector_t subvector(const Vector_t &, unsigned , unsigned )
    {
        BOOST_STATIC_ASSERT((boost::is_same<Vector_t, maal::boost::Vector>::value
                            ||boost::is_same<Vector_t, Eigen::VectorXd>::value));
    }

    template <>
    inline maal::boost::Vector subvector
        (const maal::boost::Vector& v, unsigned begin, unsigned length)
    {
        BOOST_ASSERT(v.size()<=(begin+length) && begin>=0 && "The subvector is incorrectly set.");

        maal::boost::Vector v2(length);

        for (unsigned i=0 ; i<length ; i)
        {
            v2 (i)= v(i+begin);
        }
        return v;

    }

    template <>
    inline Eigen::VectorXd subvector
        (const Eigen::VectorXd & v, unsigned begin, unsigned length)
    {
        return v.segment(begin,length);
    }


}

#endif // SOTSTATEOBSERVATIONTOOLSDEFINITIONS
