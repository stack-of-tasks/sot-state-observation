#ifndef SOTSTATEOBSERVATIONTOOLSDEFINITIONS
#define SOTSTATEOBSERVATIONTOOLSDEFINITIONS

#include <state-observation/tools/definitions.hpp>
#include <dynamic-graph/linear-algebra.h>


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

}

#endif // SOTSTATEOBSERVATIONTOOLSDEFINITIONS
