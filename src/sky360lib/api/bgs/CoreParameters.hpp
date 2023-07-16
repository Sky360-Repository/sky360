#pragma once

#include "CoreBgs.hpp"

#include <map>
#include <string>
#include <iostream>

namespace sky360lib::bgs
{
    class CoreParameters
    {
    public:
        void set_bgs(CoreBgs* _bgs)
        {
            m_core_bgs = _bgs;
        }

        friend class CoreBgs;

    protected:

        CoreBgs* m_core_bgs;
    };
}