

setup_wx_all(wxAll)
if (wxWidgets_PRECOMP)
    use_precompiled_header(wxAll ${wxROOT_INC}/wx/wxprec.h dummy.cpp)
    set(wxPRECOMPSRC dummy.cpp)
endif()
make_header(wxALLHDR ${wxALL_HDR})
make_source(wxALLSRC ${wxALL_SRC})
ADD_DEFINITIONS(-DNOPCH)
add_library(wxAll ${wxALLSRC} ${wxALLHDR} ${wxPRECOMPSRC})
