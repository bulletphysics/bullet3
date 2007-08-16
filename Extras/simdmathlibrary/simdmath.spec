%define lib_version 1.0.2

Summary: SIMD math library.
Name: simdmath
Version: %{lib_version}
Release: 1
License: BSD
Group: System Environment/Libraries
Source0: %{name}-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root

%define _enable_spu 0

%ifarch ppc
%define _arch ppu
%define _libdir %{_exec_prefix}/lib
%define _enable_spu 1
%endif

%ifarch ppc64
%define _arch ppu64
%define _libdir %{_exec_prefix}/lib64
%define _enable_spu 1
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

%if %{_enable_spu}
%package -n spu-%{name}-devel
Summary: SIMD math library.
Group: Development/Libraries
%endif

%description
SIMD math library.

%description -n %{name}-devel
SIMD math library.

%if %{_enable_spu}
%description -n spu-%{name}-devel
SIMD math library.
%endif

%prep
%setup -q

%build

make ARCH=%{_arch}

%if %{_enable_spu}
make ARCH=spu
%endif

%install
rm -rf %{buildroot}

make ARCH=%{_arch} DESTDIR=%{buildroot} install

%if %{_enable_spu}
make ARCH=spu DESTDIR=%{buildroot} install
%endif

if [ "%{_libdir}" != /usr/lib ]; then
	mv %{buildroot}/usr/lib %{buildroot}%{_libdir}
fi

mkdir -p %{buildroot}/%{_docdir}/%{name}-%{version}
cp README LICENSE %{buildroot}/%{_docdir}/%{name}-%{version}/

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root,-)
%{_libdir}/*.so.*
%{_docdir}/*

%files -n %{name}-devel
%defattr(-,root,root,-)
%{_includedir}/*
%{_libdir}/*.a
%{_libdir}/*.so

%if %{_enable_spu}
%files -n spu-%{name}-devel
%defattr(-,root,root,-)
%{_spe_includedir}/*
%{_spe_libdir}/*
%endif

%changelog
* Thu Aug  9 2007 Kazunori Asayama <asayama@sm.sony.co.jp> - 1.0.1-2
- Add documents.

* Fri Aug 25 2006 Kazunori Asayama <asayama@sm.sony.co.jp> - 1.0.1-1
- Initial build.
