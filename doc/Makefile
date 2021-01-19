## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

images_jpg := $(addprefix images/, $(addsuffix .jpg, displacement_geometry instanced_geometry pathtracer dynamic_scene subdivision_geometry user_geometry hair_geometry intersection_filter triangle_geometry viewer viewer_stream interpolation motion_blur_geometry curve_geometry voronoi closest_point multi_level_instancing point_geometry grid_geometry quaternion_motion_blur collide))
images_png := $(addprefix images/, $(addsuffix .png, quad_uv triangle_uv half_edges))
images_pdf := $(addprefix images/, $(addsuffix .pdf, quad_uv triangle_uv half_edges))

webpages := $(addprefix www/, $(addsuffix .html, index api tutorials downloads renderer related legal))
webimages := $(addprefix www/, $(images_jpg))
docfiles := $(addprefix doc/, README.md CHANGELOG.md readme.pdf)
spec := embree-spec.rst
man_src := $(wildcard src/api/*.md)
manfiles := $(patsubst src/api/%.md,man/man3/%.3embree3,$(man_src))

all: www images man doc spec
www: $(webpages) $(webimages)
doc: $(docfiles)
man: $(manfiles)
spec: $(spec)
images: $(images_jpg) $(images_png) $(images_pdf)

.PHONY: all www doc images

PANDOC := pandoc

########################################################################
### add version
########################################################################

tmp/version: ../include/embree3/rtcore_config.h
	sed -n 's/#define RTC_VERSION_STRING \"\(.*\)\".*/\1/p' $< > $@
tmp/version_major: ../include/embree3/rtcore_config.h
	sed -n 's/#define RTC_VERSION_MAJOR \([0-9]\)/\1/p' $< > $@

replace_version = sed -e "s/<EMBREE_VERSION>/`cat tmp/version`/g" $< | sed -e "s/<EMBREE_VERSION_MAJOR>/`cat tmp/version_major`/g" > $@

tmp/%.md: src/%.md tmp/version tmp/version_major
	$(replace_version)
tmp/%.html: src/%.html tmp/version tmp/version_major
	$(replace_version)

tmp/images_local_%.md: tmp/images_local.md
	sed -e "s/\.fig/.$*/" $< > $@


########################################################################
### directories
########################################################################
$(webpages): | wwwdir
wwwdir:
	@mkdir -p www
	@mkdir -p www/images

$(docfiles): | docdir
docdir:
	@mkdir -p doc

$(manfiles): | mandir
mandir:
	@mkdir -p man
	@mkdir -p man/man3

tmp/readme.tex doc/readme.pdf tmp/links_web.md tmp/metadata.md tmp/downloading.md tmp/version tmp/version_major: | tmpdir
tmpdir:
	@mkdir -p tmp


convertbsp = sed -e 's@\\\[@[@g' -e 's@\\\]@]@g' -e 's@\\_]@_@g'

########################################################################
### webpages
########################################################################

markdown2web = $(PANDOC) --email-obfuscation=none -f markdown $(filter-out tmp/webtemplate.html,$+) --template tmp/webtemplate -V select_$(basename $(@F)) -o $@ -s --mathjax=https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML
#markdown2web = $(PANDOC) $(filter-out tmp/webtemplate.html,$+) --template tmp/webtemplate -V select_$(basename $(@F)) -o $@ -s --mathjax=https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML


$(spec) $(webpages): tmp/links_web.md tmp/images_local_png.md tmp/webtemplate.html

www/%.html:
	$(markdown2web)

www/index.html: tmp/overview.md src/resources.html tmp/support.md tmp/changelog.md
www/downloads.html: tmp/platforms.md tmp/downloading.md tmp/install.md tmp/compilation.md
www/tutorials.html: tmp/tutorials.md
www/renderer.html: tmp/renderer.md
www/related.html: tmp/projects.md src/papers.html
www/legal.html: tmp/legal.md
	$(markdown2web) --indented-code-classes=cpp

www/api.html: tmp/api.md
	$(PANDOC) $(filter-out tmp/webtemplate.html,$+) --filter src/IncludeFilter.py -t markdown | $(convertbsp) > tmp/api_webinc.md
	$(PANDOC) --email-obfuscation=none -f markdown --template tmp/webtemplate tmp/api_webinc.md  tmp/links_web.md tmp/images_local_png.md --indented-code-classes=cpp -V select_$(basename $(@F)) -o $@

embree-spec.rst: tmp/spec.md
	$(PANDOC) $(filter-out tmp/webtemplate.html,$+) --filter src/IncludeFilter.py -t markdown | $(convertbsp) > tmp/spec_webinc.md
	$(PANDOC) --email-obfuscation=none -f markdown tmp/spec_webinc.md  tmp/links_web.md tmp/images_local_png.md --indented-code-classes=cpp -V select_$(basename $(@F)) --to rst -o embree-spec.rst


www/images/%.jpg: images/%.jpg
	cp $< $@ 

########################################################################
### images
########################################################################

images/%.jpg:
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -size 512 512 -o images/`basename -s .jpg $@`.tga
	convert images/`basename -s .jpg $@`.tga images/`basename -s .jpg $@`.jpg
	rm images/`basename -s .jpg $@`.tga

images/viewer.jpg:
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -c ~/Work/models/embree/crown/crown_paper.ecs -size 512 512 -frames 0 16 -o images/`basename -s .jpg $@`.tga
	convert images/`basename -s .jpg $@`.tga images/`basename -s .jpg $@`.jpg
	rm images/`basename -s .jpg $@`.tga

images/pathtracer.jpg:
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -c ~/Work/models/embree/crown/crown_paper.ecs -size 512 512 -frames 0 16 -o images/`basename -s .jpg $@`.tga
	convert images/`basename -s .jpg $@`.tga images/`basename -s .jpg $@`.jpg
	rm images/`basename -s .jpg $@`.tga

images/%.png: images/%.fig
	fig2dev -L png -S 4 $+ $@

images/%.pdf: images/%.fig
	fig2dev -L pdf $+ $@


########################################################################
### markdown
########################################################################

convertnbsppagebreak = sed -e 's@\\ @ @g' -e 's@\\pagebreak@@g'

pandoc_version_major := $(shell $(PANDOC) --version | head -n 1 | sed -e 's/[^0-9]*\([0-9]*\).*/\1/')
ifeq ($(pandoc_version_major),1)
top_level_chapter = --chapters
else
top_level_chapter = --top-level-division=chapter
endif

tmp/api_inc.md: tmp/api.md
	$(PANDOC) --columns=71 $+ --filter src/IncludeFilter.py -t markdown | $(convertbsp) > $@

doc/README.md: tmp/metadata.md tmp/overview.md tmp/platforms.md tmp/support.md tmp/install.md tmp/compilation.md tmp/api_inc.md tmp/tutorials.md tmp/links_local.md tmp/images_url.md
	cat $+ | $(convertnbsppagebreak) > $@

doc/CHANGELOG.md: tmp/changelog.md
	$(convertnbsppagebreak) $+ > $@

########################################################################
### pdf
########################################################################

tmp/readme_all.md: tmp/overview.md tmp/platforms.md tmp/support.md tmp/changelog.md tmp/install.md tmp/compilation.md tmp/api_inc.md tmp/tutorials.md tmp/links_local.md tmp/images_local_pdf.md
	cat $+ > $@

tmp/readme.tex: src/readme.tex
	cp src/readme.tex tmp

tmp/%.tex: tmp/%.md
	$(PANDOC) $+ --indented-code-classes=cpp -t latex --columns=72 $(top_level_chapter) -o $@

doc/readme.pdf: preamble.tex tmp/version

doc/readme.pdf: tmp/readme.tex tmp/readme_all.tex $(images_jpg) $(images_pdf)
	xelatex -output-directory=tmp $<
	xelatex -output-directory=tmp $<
	cp tmp/readme.pdf $@

########################################################################
### manpages
########################################################################

man/man3/%.3embree3: src/api/%.md
	$(PANDOC) -s -t man $< -o $@

########################################################################
### clean
########################################################################

clean:
	rm -rf doc tmp man www embree-spec.rst
