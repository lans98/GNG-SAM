#ifndef GNG_SAM_TOOLS_HPP
#define GNG_SAM_TOOLS_HPP

#include <string>
#include <stdexcept>

namespace gng {
namespace tools {
    
    using namespace std;    

    void expect(bool condition, string msg) { if (!condition) throw runtime_error(msg); }
}
}

#endif 
