%define major_version 1
%define minor_version 0

Summary: SIMD math library.
Name: simdmath
Version: %{major_version}.%{minor_version}
Release: 1
License: BSD
Group: System Environment/Libraries
Source0: %{name}-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root

%ifarch ppc
%define _arch 32
%define _libdir %{_exec_prefix}/lib
%endif

%ifarch ppc64
%define _arch 64
%define _libdir %{_exec_prefix}/lib64
%endif

%define _spe_prefix %{_prefix}/spu
%define _spe_exec_prefix %{_exec_prefix}/spu
%define _spe_includedir %{_spe_prefix}/include
%define _spe_libdir %{_spe_exec_prefix}/lib

%define __os_install_post	%{nil}

%package -n %{name}-devel
Summary: SIMD math library.
Group: Development/Libraries
Requires: %{name} = %{version}

%package -n %{name}-spe-devel
Summary: SIMD math library.
Group: Development/Libraries

%description
SIMD math library.

%description -n %{name}-devel
SIMD math library.

%description -n %{name}-spe-devel
SIMD math library.

%prep
%setup -q

%build

make CROSS_PPU= ARCH_PPU=%{_arch}

%install
rm -rf %{buildroot}

make CROSS_PPU= ARCH_PPU=%{_arch} DESTDIR=%{buildroot} install

if [ "%{_libdir}" != /usr/lib ]; then
	mv %{buildroot}/usr/lib %{buildroot}%{_libdir}
fi

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root,-)
%{_libdir}/*.so.*

%files -n %{name}-devel
%defattr(-,root,root,-)
%{_includedir}/*
%{_libdir}/*.a
%{_libdir}/*.so

%files -n %{name}-spe-devel
%defattr(-,root,root,-)
%{_spe_includedir}/*
%{_spe_libdir}/*

%changelog
* Fri Aug 25 2006 Kazunori Asayama <asayama@sm.sony.co.jp> - 
- Initial build.

