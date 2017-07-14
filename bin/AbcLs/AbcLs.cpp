//-*****************************************************************************
//
// Copyright (c) 2013,
//  Sony Pictures Imageworks, Inc. and
//  Industrial Light & Magic, a division of Lucasfilm Entertainment Company Ltd.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Sony Pictures Imageworks, nor
// Industrial Light & Magic nor the names of their contributors may be used
// to endorse or promote products derived from this software without specific
// prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-*****************************************************************************

#include <Alembic/Abc/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#ifdef ALEMBIC_WITH_HDF5
#include <Alembic/AbcCoreHDF5/All.h>
#endif
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCollection/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcMaterial/All.h>

#include <signal.h>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <sys/stat.h>

// no unistd under visual studio
#ifndef _MSC_VER
#include <unistd.h>
#endif

#ifdef _MSC_VER
#include <locale>         // std::locale, std::isdigit
// set up _S_ISDIR()
#if !defined(S_ISDIR)
#  if defined( _S_IFDIR) && !defined( __S_IFDIR)
#    define __S_IFDIR _S_IFDIR
#  endif
#  define S_ISDIR(mode)    (mode&__S_IFDIR)
#endif
#endif // _MSC_VER

namespace Abc  = ::Alembic::Abc;
namespace AbcA = ::Alembic::AbcCoreAbstract;
namespace AbcF = ::Alembic::AbcCoreFactory;
namespace AbcG = ::Alembic::AbcGeom;
namespace AbcU = ::Alembic::Util;

using AbcA::index_t;

void segfault_sigaction(int signal)
{
    std::cout << "Unrecoverable error: signal " << signal << std::endl;
    exit(0);
}

//-*****************************************************************************
// overload to print as a number instead of a character
std::ostream & operator<<(std::ostream & os, Alembic::Util::uint8_t val)
{
    return os << static_cast<int>(val);
}

//-*****************************************************************************
// overload to print as a number instead of a character
std::ostream & operator<<(std::ostream & os, Alembic::Util::int8_t val)
{
    return os << static_cast<int>(val);
}

//-*****************************************************************************
bool is_digit( const std::string& s )
{
    std::locale loc;
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it,loc)) ++it;
    return !s.empty() && it == s.end();
}

//-*****************************************************************************
void  printTimeSampling( AbcA::TimeSamplingPtr iTime, index_t iMaxSample,
                         double fps )
{
    AbcA::TimeSamplingType timeType = iTime->getTimeSamplingType();
    if ( timeType.isUniform() ) {
        std::cout << "    \"type\": \"uniform\"," << std::endl;
        std::cout << "    \"start_time\": " << iTime->getStoredTimes()[0] * fps << "," << std::endl;
        std::cout << "    \"time_per_cycle\": " << timeType.getTimePerCycle() * fps << "," << std::endl;
    }
    else if ( timeType.isCyclic() ) {
        std::cout << "    \"type\": \"cyclic\", " << std::endl;
        std::cout << "    \"time_per_cycle\": " << timeType.getTimePerCycle() * fps << "," << std::endl;

        const std::vector < double > & storedTimes = iTime->getStoredTimes();
        std::size_t numTimes = iTime->getNumStoredTimes();
        std::cout << "    \"start_cycle_times\": [";
        for (std::size_t i = 0; i < numTimes; ++i ) {
            if (i != 0) {
                std::cout << ", ";
            }
            std::cout << storedTimes[i] * fps;
        }
        std:: cout << "]," << std::endl;
    }
    else {
        std::cout << "    \"type\":\"acyclic\"," << std::endl;

        const std::vector < double > & storedTimes = iTime->getStoredTimes();
        std::size_t numTimes = iTime->getNumStoredTimes();

        std::cout << "    \"times\": [";
        for (std::size_t i = 0; i < numTimes; ++i ) {
            if (i != 0) {
                std::cout << ", ";
            }
            std::cout << storedTimes[i] * fps;
        }
        std:: cout << "]," << std::endl;
    }

    std::cout << "    \"max_num_samples\": " << iMaxSample << std::endl;
}

//-*****************************************************************************
void printParent( Abc::ICompoundProperty iProp,
                  bool all = false,
                  bool long_list = false,
                  bool recursive = false,
                  bool first = false )
{
    std::cout << "  \"" << iProp.getObject().getFullName() << "/" <<
        iProp.getName() << "\": " << std::endl;
}

//-*****************************************************************************
void printParent( AbcG::IObject iObj,
                  bool all = false,
                  bool long_list = false,
                  bool recursive = false,
                  bool first = false )
{
    if ( !first && !long_list )
        std::cout << std::endl;

    std::cout << "  \"" << iObj.getFullName() << "\":" << std::endl;
}

//-*****************************************************************************
void printMetaData( AbcA::MetaData md )
{
    std::stringstream ss( md.serialize() );
    std::string segment;

    std::cout << "    \"metadata\": {";

    for (AbcA::MetaData::const_iterator it = md.begin(); it != md.end(); ++it) {
        if (it != md.begin()) {
            std::cout << ", ";
        }
        std::cout << "\"" << it->first << "\":\"" << it->second << "\"";
    }
    std::cout << "}" << std::endl;
}

//-*****************************************************************************
template<class TPTraits>
void getScalarValue( Abc::IScalarProperty &p, const Abc::ISampleSelector &iSS )
{
    typedef typename TPTraits::value_type U;
    std::size_t extent = p.getDataType().getExtent();
    std::vector< U > val( extent );
    p.get( reinterpret_cast<void*>( &val.front() ), iSS );

    std::cout << "[";

    bool isStr = (p.getDataType().getPod() == AbcU::kStringPOD) ||
        (p.getDataType().getPod() == AbcU::kWstringPOD);

    for ( std::size_t i = 0; i < extent; ++i ) {

        if ( i != 0 ) {
            std::cout << ", ";
        }


        if ( isStr ) {
            std::cout << "\"";
        }

        std::cout << val[i];

        if ( isStr ) {
            std::cout << "\"";
        }
    }

    std::cout << "]" << std::endl;
}

//-*****************************************************************************
#define CASE_RETURN_SCALAR_VALUE( TPTraits, PROP, SELECTOR )    \
case TPTraits::pod_enum:                                        \
    return getScalarValue<TPTraits>( PROP, SELECTOR );

//-*****************************************************************************
template<class TYPE>
void getArrayValue( Abc::IArrayProperty &p, Abc::PropertyHeader &header,
                       const Abc::ISampleSelector &iSS )
{
    Abc::DataType dt = header.getDataType();
    AbcU ::uint8_t extent = dt.getExtent();

    bool isStr = (dt.getPod() == AbcU::kStringPOD) ||
        (dt.getPod() == AbcU::kWstringPOD);

    Abc::ArraySamplePtr ptr;
    p.get( ptr, iSS );
    size_t totalValues = ptr->getDimensions().numPoints() * extent;
    TYPE *vals = (TYPE*)(ptr->getData());
    std::cout << "[ ";
    for ( size_t i=0; i<totalValues; ++i ) {

        if (isStr) {
            std::cout << "\"";
        }

        std::cout << vals[i];

        if (isStr) {
            std::cout << "\"";
        }

        if ( (i + 1) % extent == 0 ) {
            std::cout << std::endl;
        }
        else if (totalValues != 1) {
            std::cout << ", ";
        }
    }
    std::cout << " ]" << std::endl;
}

//-*****************************************************************************
#define CASE_RETURN_ARRAY_VALUE( TPTraits, TYPE, PROP, HEADER, SELECTOR ) \
case TPTraits::pod_enum:                                         \
    return getArrayValue<TYPE>( PROP, HEADER, SELECTOR );

//-*****************************************************************************
void printValue( Abc::ICompoundProperty iParent, Abc::PropertyHeader header,
                 int index, bool justSize, bool printTime, double fps )
{
    Abc::ISampleSelector iss( (index_t) index );
    Abc::DataType dt = header.getDataType();
    AbcU::PlainOldDataType pod = dt.getPod();
    AbcU ::uint8_t extent = dt.getExtent();

    if ( header.isArray() ) {
        Abc::IArrayProperty p( iParent, header.getName() );

        if ( printTime ) {
            std::cout << "  \"time\": " <<
                p.getTimeSampling()->getSampleTime( index ) * fps << "," << std::endl;
        }

        if ( justSize ) {
            AbcU::Dimensions dims;
            p.getDimensions( dims, iss );
            std::cout << "  \"extent\": " << (int) extent << ", ";
            std::cout << "  \"num_points\": " << dims.numPoints() << std::endl;
            return;
        }

        switch ( pod )
        {
            CASE_RETURN_ARRAY_VALUE(Abc::BooleanTPTraits, bool, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Uint8TPTraits, uint8_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Int8TPTraits, int8_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Uint16TPTraits, uint16_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Int16TPTraits, int16_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Uint32TPTraits, uint32_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Int32TPTraits, int32_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Uint64TPTraits, uint64_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Int64TPTraits, int64_t, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Float32TPTraits, float, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::Float64TPTraits, double, p, header, iss);
            CASE_RETURN_ARRAY_VALUE(Abc::StringTPTraits, std::string, p, header, iss);
            default:
                std::cout << "  \"unsupported_pod\": " << pod << std::endl;
                break;
        }
    } else if ( header.isScalar() ) {
        Abc::IScalarProperty p( iParent, header.getName() );

        if ( printTime ) {
            std::cout << "  \"time\": " <<
                p.getTimeSampling()->getSampleTime( index ) * fps << ", " << std::endl;
        }

        if ( justSize ) {
            std::cout << "  \"extent\": " << (int) extent << std::endl;
            return;
        }

        switch ( pod )
        {
            CASE_RETURN_SCALAR_VALUE( Abc::BooleanTPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Uint8TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Int8TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Uint16TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Int16TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Uint32TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Int32TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Uint64TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Int64TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Float32TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::Float64TPTraits, p, iss );
            CASE_RETURN_SCALAR_VALUE( Abc::StringTPTraits, p, iss );
            default:
                std::cout << "  \"unsupported_pod\": " << pod << std::endl;
                break;
        }

    }
}

//-*****************************************************************************
void printChild( Abc::ICompoundProperty iParent, Abc::PropertyHeader header,
                 bool all = false, bool long_list = false, bool meta = false,
                 bool values = false )
{
    std::cout << "  \"" << header.getName() << "\": {";
    if ( long_list ) {
        if ( header.isCompound() ) {
            std::cout << "    \"property_type\": \"compound\", ";
        } else if ( header.isScalar() ) {
            std::cout << "    \"property_type\": \"scalar\", ";
            std::cout << "    \"data_type\": \"" << header.getDataType() << "\", ";
        } else if ( header.isArray() ) {
            std::cout << "    \"property_type\": \"array\", ";
            std::cout << "    \"data_type\": \"" << header.getDataType() << "\", ";
        }
    }

    if ( long_list ) {
        if ( header.isScalar() ) {
            Abc::IScalarProperty iProp( iParent, header.getName() );
            if (iProp.isConstant()) {
                std::cout << "    \"constant\": 1," << std::endl;
            }
            std::cout << "    \"num_samples\": " << iProp.getNumSamples() << ", " << std::endl;
        } else if ( header.isArray() ) {
            Abc::IArrayProperty iProp( iParent, header.getName() );
            if (iProp.isConstant()) {
                std::cout << "    \"constant\": 1," << std::endl;
            }
            std::cout << "    \"num_samples\": " << iProp.getNumSamples() << ", " << std::endl;
        }

        if ( meta ) {
            printMetaData(header.getMetaData());
        }
    }

    // try to access and print the 0th sample value
    if ( values && !header.isCompound() ) {
        try {
            printValue( iParent, header, 0, false, false, 24.0 );
        } catch ( std::exception& e ) {
            std::cerr << "Exception : " << e.what() << std::endl;
        }
    }

    std::cout << "  }";
}

//-*****************************************************************************
void printChild( AbcG::IObject iParent, AbcG::IObject iObj,
                 bool all = false, bool long_list = false, bool meta = false,
                 bool values = false )
{

    std::cout << "  \""<< iObj.getName() << "\": {" << std::endl;

    if ( meta ) {
        printMetaData( iObj.getMetaData());
    }
    else {
        AbcA::MetaData md = iObj.getMetaData();
        std::cout << "    \"metadata\": { \"schema\": \"" << md.get( "schema" ) << "\" }" << std::endl;
    }

    std::cout << "  }" << std::endl;

}

//-*****************************************************************************
void visit( Abc::ICompoundProperty iProp,
            bool all = false,
            bool long_list = false,
            bool meta = false,
            bool recursive = false,
            bool first = false,
            bool values = false )
{
    std::cout << "  \"props\": {" << std::endl;

    // header
    if ( recursive && iProp.getNumProperties() > 0 ) {
        printParent( iProp, all, long_list, recursive, first );
    }

    // children
    for( size_t c = 0; c < iProp.getNumProperties(); ++c ) {
        printChild( iProp, iProp.getPropertyHeader( c ), all, long_list, meta,
                    values );
    }

    // visit children
    if ( recursive && all && iProp.getNumProperties() > 0 ) {
        for( size_t p = 0; p < iProp.getNumProperties(); ++p ) {
            Abc::PropertyHeader header = iProp.getPropertyHeader( p );
            if ( header.isCompound() )
                visit( Abc::ICompoundProperty( iProp, header.getName() ),
                       all, long_list, meta, recursive, false, values );
        }
    }

    std::cout << "  }" << std::endl;
}

//-*****************************************************************************
void visit( AbcG::IObject iObj,
            bool all = false,
            bool long_list = false,
            bool meta = false,
            bool recursive = false,
            bool first = false,
            bool values = false )

{
    Abc::ICompoundProperty props = iObj.getProperties();

    // header
    if ( recursive &&
       ( iObj.getNumChildren() > 0 ||
       ( all && props.getNumProperties() > 0 ) ) ) {
        printParent( iObj, all, long_list, recursive, first );
    }

    // children
    for( size_t c = 0; c < iObj.getNumChildren(); ++c ) {
        printChild( iObj, iObj.getChild( c ), all, long_list, meta, values );
    }

    // properties
    if ( all ) {
        std::cout << "  \"props\": {" << std::endl;
        for( size_t h = 0; h < props.getNumProperties(); ++h ) {
            printChild( props, props.getPropertyHeader( h ), all, long_list, meta, values );
        }
        std::cout << "  }" << std::endl;
    }

    // visit property children
    if ( recursive && all && props.getNumProperties() > 0 ) {
        for( size_t p = 0; p < props.getNumProperties(); ++p ) {
            Abc::PropertyHeader header = props.getPropertyHeader( p );
            if ( header.isCompound() ) {
                if ( !long_list )
                    std::cout << std::endl;
                visit( Abc::ICompoundProperty( props, header.getName() ),
                       all, long_list, meta, recursive, false, values );
            }
        }
    }

    // visit object children
    if ( recursive && iObj.getNumChildren() > 0 ) {
        for( size_t c = 0; c < iObj.getNumChildren(); ++c ) {
            visit( iObj.getChild( c ), all, long_list, meta, recursive, false, values );
        }
    }
}

//-*****************************************************************************
bool isFile( const std::string& filename )
{
    struct stat buf;
    if ( stat(filename.c_str(), &buf) == 0 && !S_ISDIR( buf.st_mode ) ) {
        return true;
    }
    return false;
}

//-*****************************************************************************
bool optionExists(std::vector<std::string> options, std::string option)
{
    for ( std::size_t i = 0; i < options.size(); i++ )
        if ( options[i].find(option) != std::string::npos )
            return true;
    return false;
}

//-*****************************************************************************
int main( int argc, char *argv[] )
{
    //float opt_fps = 24.0;
    bool opt_all = false; // show all option
    bool opt_long = false; // long listing option
    bool opt_meta = false; // metadata option
    bool opt_recursive = false; // recursive option
    bool opt_size = false; // array sample size option
    bool opt_time = false; // time info option
    bool opt_values = false; // show all 0th values
    int index = -1; // sample number, at tail of path
    std::string desc( "abcls [OPTION] FILE[/NAME] \n"
    "  -a          include property listings\n"
    "  -f          show time sampling as 24 fps\n"
    "  -h, --help  show this help message\n"
    "  -l          long listing format\n"
    "  -m          show archive metadata\n"
    "  -r          list entries recursively\n"
    "  -s          show the size of a data property sample\n"
    "  -t          show time sampling information\n"
    "  -v          show 0th value for all properties\n"
    );

    /* sigaction if available */
#if defined(_POSIX_VERSION) && (_POSIX_VERSION >= 199506L)
    // seg fault handler
    struct sigaction act;
    sigemptyset(&act.sa_mask);
    act.sa_handler = segfault_sigaction;
    act.sa_flags = SA_SIGINFO;
    sigaction(SIGSEGV, &act, NULL);
    /* signal if available */
#elif defined(_POSIX_VERSION) || defined(_MSC_VER)
    signal(SIGSEGV, segfault_sigaction);
#else
#error No signal interface available
#endif //_POSIX_VERSION

    // check for min args
    if ( argc < 2 ) {
        std::cout << desc << std::endl;
        return 0;
    };

    // parse args
    std::vector<std::string> arguments(argv, argv + argc);
    std::vector<std::string> options;
    std::vector<std::string> files;

    // separate file args from option args
    for ( std::size_t i = 1; i < arguments.size(); i++ ) {
        if ( arguments[ i ].substr( 0, 1 ) == "-" )
            options.push_back( arguments[ i ] );
        else
            files.push_back( arguments[ i ] );
    }

    // help
    if ( argc < 2 ||
         optionExists( options, "h" ) ||
         optionExists( options, "help" )
       ) {
        std::cout << desc << std::endl;
        return 0;
    };

    // set some flags
    double fps = 1.0;
    opt_all = optionExists( options, "a" );
    opt_long = optionExists( options, "l" );
    opt_meta = optionExists( options, "m" );
    opt_recursive = optionExists( options, "r" );
    opt_size = optionExists( options, "s" );
    opt_time = optionExists( options, "t" );
    opt_values = optionExists( options, "v" );
    if ( optionExists( options, "f" ) ) {
        fps = 24.0;
        opt_time = true;
    }

    // open each file
    for ( std::size_t i = 0; i < files.size(); i++ ) {
        if ( files.size() > 1 )
            std::cout << "\"file\": \"" << files[i] << "\"" << std::endl;

        std::stringstream ss( files[i] );
        std::stringstream fp;
        std::string segment;
        std::vector<std::string> seglist;

        /*
         * separate file and object paths, e.g.
         *
         *   ../dir1/foo.abc/bar/baz/index
         *   \_____________/\______/\____/
         *        file         obj   sample
         */
        int j = 0;
        while ( std::getline( ss, segment, '/' ) ) {
            if ( !isFile ( fp.str() ) ) {
                if ( j != 0 )
                    fp << "/";
                fp << segment;
            } else {
                seglist.push_back( segment );
            }
            ++j;
        }

        bool lastIsIndex = false;
        if (!seglist.empty() && is_digit( seglist.back() ) ) {
            index = atoi( seglist.back().c_str() );
            lastIsIndex = true;
        }

        // open the iarchive
        Abc::IArchive archive;
        AbcF::IFactory factory;
        factory.setPolicy(Abc::ErrorHandler::kQuietNoopPolicy);
        AbcF::IFactory::CoreType coreType;
        archive = factory.getArchive(std::string( fp.str() ), coreType);

        std::cout << "{" << std::endl;

        // display file metadata
        if ( opt_meta && seglist.size() == 0 ) {
            std::cout << "  \"library_version\": \""
                << Alembic::AbcCoreAbstract::GetLibraryVersion() << "\", "
                << std::endl;

            std::string appName;
            std::string libraryVersionString;
            Alembic::Util::uint32_t libraryVersion;
            std::string whenWritten;
            std::string userDescription;
            std::string coreName;
            GetArchiveInfo (archive,
                            appName,
                            libraryVersionString,
                            libraryVersion,
                            whenWritten,
                            userDescription);

            if ( coreType == AbcF::IFactory::kOgawa ) {
                coreName = "\"Ogawa\"";
            } else if ( coreType == AbcF::IFactory::kHDF5 ) {
                coreName = "\"HDF5\"";
            } else {
                coreName = "\"Unknown\"";
            };

            if ( appName != "" ) {
                std::cout << "  \"app_name\": \"" << appName << "\", " << std::endl;
                std::cout << "  \"lib_ver\": \"" << libraryVersionString << "\"," << std::endl;
                std::cout << "  \"date_written\": \"" << whenWritten << "\"," << std::endl;
                std::cout << "  \"user_desc\": \"" << userDescription << "\"," << std::endl;
            }
            std::cout << "  \"core_type\": " << coreName << "," << std::endl;
        };

        if ( opt_time && seglist.size() == 0 ) {
            std::cout << "  \"time_sampling\": {" << std::endl;
            uint32_t numTimes = archive.getNumTimeSamplings();
            for ( uint32_t k = 0; k < numTimes; ++k ) {
                AbcA::TimeSamplingPtr ts = archive.getTimeSampling( k );
                index_t maxSample =
                    archive.getMaxNumSamplesForTimeSamplingIndex( k );
                std::cout << "    \"" << k << "\": { ";
                printTimeSampling( ts, maxSample, fps );
                if (k != numTimes - 1) {
                    std::cout << "    }," << std::endl;
                }
                else {
                    std::cout << "    }" << std::endl;
                }
            }
            std::cout << "  }" << std::endl;

        }

        // walk object hierarchy and find valid objects
        AbcG::IObject test = archive.getTop();
        AbcG::IObject iObj = test;
        while ( test.valid() && seglist.size() > 0 ) {
            test = test.getChild( seglist.front() );
            if ( test.valid() ) {
                iObj = test;
                seglist.erase( seglist.begin() );
            }
        }

        // walk property hierarchy for most recent object
        Abc::ICompoundProperty props = iObj.getProperties();
        const Abc::PropertyHeader* header;
        bool found = false;
        bool shouldPrintValue = false;
        for ( std::size_t i = 0; i < seglist.size(); ++i ) {
            header = props.getPropertyHeader( seglist[i] );
            if ( header && header->isCompound() ) {
                Abc::ICompoundProperty ptest( props, header->getName() );
                if ( ptest.valid() ) {
                    props = ptest;
                    found = true;
                }
            } else if ( header && header->isSimple() ) {
                found = true;

                // if the last value happens to be an index, and we are a
                // property  then dont bother checking the last item in seglist
                if (lastIsIndex && i == seglist.size() - 2)
                {
                    shouldPrintValue = true;
                    break;
                }
            } else {
                std::cout << "\"" << seglist[i] << "\": \"invalid\"" << std::endl;
                return 1;
            }
        }

        // do stuff
        if ( shouldPrintValue ) {
            printValue( props, *header, index, opt_size, opt_time, fps );
        } else {
            if ( found && header->isCompound() )
                visit( props, opt_all, opt_long, opt_meta, opt_recursive, true, opt_values );
            else if ( found && header->isSimple() )
                printChild( props, *header, opt_all, opt_long, opt_values );
            else
                visit( iObj, opt_all, opt_long, opt_meta, opt_recursive, true, opt_values );
        }
    }

    std::cout << "}" << std::endl;
    return 0;
}
