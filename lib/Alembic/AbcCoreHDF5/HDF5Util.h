//-*****************************************************************************
//
// Copyright (c) 2009-2012,
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

#ifndef Alembic_AbcCoreHDF5_HDF5Util_h
#define Alembic_AbcCoreHDF5_HDF5Util_h

#include <Alembic/AbcCoreHDF5/Foundation.h>
#include <Alembic/AbcCoreHDF5/HDF5Hierarchy.h>

namespace Alembic {
namespace AbcCoreHDF5 {
namespace ALEMBIC_VERSION_NS {

//-*****************************************************************************
typedef ::Alembic::Util::BaseDimensions<hsize_t> HDimensions;

//-*****************************************************************************
//! RAII for HDF attributes
struct AttrCloser
{
    AttrCloser( hid_t id ) : m_id( id ) {}
    ~AttrCloser() { if ( m_id >= 0 ) H5Aclose( m_id ); }
    hid_t m_id;
};

//-*****************************************************************************
//! RAII for HDF data space
struct DspaceCloser
{
    DspaceCloser( hid_t id ) : m_id( id ) {}
    ~DspaceCloser() { if ( m_id >= 0 ) H5Sclose( m_id ); }
    hid_t m_id;
};

//-*****************************************************************************
//! RAII for HDF data set
struct DsetCloser
{
    DsetCloser( hid_t id ) : m_id( id ) {}
    ~DsetCloser() { if (m_id >= 0 ) H5Dclose( m_id ); }
    hid_t m_id;
};


//-*****************************************************************************
//! RAII for HDF Group
struct GroupCloser
{
    GroupCloser( hid_t id ) : m_id( id ) {}
    ~GroupCloser() { if ( m_id >= 0 ) H5Gclose( m_id ); }
    hid_t m_id;
};

//-*****************************************************************************
//! RAII for HDF data type
struct DtypeCloser
{
    DtypeCloser( hid_t id ) : m_id( id ) {}
    ~DtypeCloser() { if ( m_id >= 0 ) H5Tclose( m_id ); }
    hid_t m_id;
};

//-*****************************************************************************
//! RAII for HDF property
struct PlistCloser
{
    PlistCloser( hid_t id ) : m_id( id ) {}
    ~PlistCloser() { if ( m_id >= 0 ) H5Pclose( m_id ); }
    hid_t m_id;
};

//-*****************************************************************************
//! utility which creates an HDF5 property and makes sure the order is tracked
hid_t CreationOrderPlist();

//! utility which creates a property and will gzip it when written to
hid_t DsetGzipCreatePlist( const Dimensions &dims, int level );

//-*****************************************************************************
//! utility which compares 2 HDF5 data types and returns if they are the same
bool EquivalentDatatypes( hid_t idA, hid_t idB );

//-*****************************************************************************
//! utility which opens a group if it is being reference or is a child group
H5Node OpenGroup( H5Node& iParent, const std::string& iName );

//-*****************************************************************************
//! utility which closes an HDF5 object
void CloseObject (H5Node& iNode );

//-*****************************************************************************
//! utility which determines if the HDF5 child group or link exists
bool GroupExists( H5Node& iParent, const std::string &iName );

//-*****************************************************************************
//! utility which determines if the HDF5 child object or link exists
bool ObjectExists( H5Node& iParent, const std::string &iName );

//-*****************************************************************************
//! utility which determines if the HDF5 child attribute exists
bool AttrExists( H5Node& iParent, const std::string &iName );

//-*****************************************************************************
//! utility which determines if the HDF5 child dataset or link exists
bool DatasetExists( H5Node& iParent, const std::string &iName );

} // End namespace ALEMBIC_VERSION_NS

using namespace ALEMBIC_VERSION_NS;

} // End namespace AbcCoreHDF5
} // End namespace Alembic

#endif
