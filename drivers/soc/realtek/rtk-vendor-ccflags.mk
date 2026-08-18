# Legacy Realtek vendor drivers: suppress benign diagnostics on imported code.
rtk_vendor-ccflags := \
	-Wno-missing-prototypes \
	-Wno-missing-declarations \
	-Wno-unused-variable \
	-Wno-unused-function \
	-Wno-unused-label \
	-Wno-empty-body \
	-Wno-dangling-else \
	-Wno-old-style-declaration \
	-Wno-duplicate-decl-specifier \
	-Wno-vla \
	-Wno-sequence-point \
	-Wno-parentheses \
	-Wno-misleading-indentation \
	-Wno-discarded-qualifiers \
	-Wno-attribute-warning \
	-Wno-format \
	-Wno-int-conversion
