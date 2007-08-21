%define lib_version 1.0.1

Summary: Vector math library
Name: vectormath
Version: %{lib_version}
Release: 1
License: BSD
Group: Development/Libraries
Source0: %{name}-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root

%description
Vector math library.

%ifarch ppc ppc64
%define _lib_arch ppu
%endif

%ifarch i386 x86_64
%define _lib_arch SSE
%endif

%if %{undefined _lib_arch}
%define _lib_arch scalar
%endif

%package -n %{name}-devel
Summary: Vector math library.
Group: Development/Libraries
%ifarch ppc ppc64
Requires: simdmath-devel
%endif

%description -n %{name}-devel
Vector math library.

%ifarch ppc ppc64
%package -n spu-%{name}-devel
Summary: Vector math library.
Group: Development/Libraries
Requires: spu-simdmath-devel

%description -n spu-%{name}-devel
Vector math library.
%endif

%prep
%setup -q

%build

%install
rm -rf %{buildroot}

make ARCH=%{_lib_arch} DESTDIR=%{buildroot} install

%ifarch ppc ppc64
make ARCH=spu DESTDIR=%{buildroot} install
%endif

mkdir -p %{buildroot}/%{_docdir}/%{name}-%{version}
cp README LICENSE doc/*.pdf %{buildroot}/%{_docdir}/%{name}-%{version}/


%clean
rm -rf %{buildroot}

%files -n %{name}-devel
%defattr(-,root,root,-)
%{_includedir}/*
%{_docdir}/*

%ifarch ppc ppc64
%files -n spu-%{name}-devel
%defattr(-,root,root,-)
%{_prefix}/spu/include/*
%endif

%changelog
* Wed Aug  8 2007 Kazunori Asayama <asayama@sm.sony.co.jp> - 1.0.1-1
- Initial build.
