images_jpg := $(addprefix images/, $(addsuffix .jpg, displacement_geometry instanced_geometry pathtracer dynamic_scene subdivision_geometry user_geometry hair_geometry intersection_filter triangle_geometry viewer viewer_stream interpolation motion_blur_geometry curve_geometry))
images_png := $(addprefix images/, $(addsuffix .png, quad_uv triangle_uv half_edges))
images_pdf := $(addprefix images/, $(addsuffix .pdf, quad_uv triangle_uv half_edges))

webpages := $(addprefix www/, $(addsuffix .html, index api tutorials downloads renderer related legal))
webimages := $(addprefix www/, $(images_jpg))
docfiles := $(addprefix doc/, README.md CHANGELOG.md readme.pdf)

man_src := $(wildcard src/api/*.md)
manfiles := $(patsubst src/api/%.md,man/man3/%.3embree3,$(man_src))

all: www images man doc
www: $(webpages) $(webimages)
doc: $(docfiles)
man: $(manfiles)
images: $(images_jpg) $(images_png) $(images_pdf)

.PHONY: all www doc images

PANDOC := pandoc

########################################################################
### add version
########################################################################

tmp/version: ../include/embree3/rtcore_version.h
	sed -n 's/#define RTC_VERSION_STRING \"\(.*\)\".*/\1/p' $< > $@
tmp/version_major: ../include/embree3/rtcore_version.h
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


########################################################################
### webpages
########################################################################

markdown2web = $(PANDOC) --smart --email-obfuscation=none -f markdown $(filter-out tmp/webtemplate.html,$+) --template tmp/webtemplate -V select_$(basename $(@F)) -o $@

markdown2web2 = $(PANDOC) $(filter-out tmp/webtemplate.html,$+) --filter src/IncludeFilter.py -t markdown | $(PANDOC) --smart --email-obfuscation=none -f markdown --template tmp/webtemplate -V select_$(basename $(@F)) -o $@

$(webpages): tmp/links_web.md tmp/images_local_png.md tmp/webtemplate.html

www/%.html:
	$(markdown2web)

www/index.html: tmp/overview.md src/resources.html tmp/support.md tmp/changelog.md tmp/acknowledgement.md
www/downloads.html: tmp/platforms.md tmp/downloading.md tmp/install.md tmp/compilation.md
www/tutorials.html: tmp/tutorials.md
www/renderer.html: tmp/renderer.md
www/related.html: tmp/projects.md src/papers.html
www/legal.html: tmp/legal.md
	$(markdown2web) --indented-code-classes=cpp

www/api.html: tmp/api.md
	$(markdown2web2) --indented-code-classes=cpp

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

convertnbsp = sed -e 's@\\ @ @g'
convertpagebreak = sed -e 's@\\pagebreak@@g'

doc/README.md: tmp/metadata.md tmp/overview.md tmp/platforms.md tmp/support.md tmp/acknowledgement.md tmp/install.md tmp/compilation.md tmp/api.md tmp/tutorials.md tmp/links_local.md tmp/images_url.md
	cat $+ | $(convertnbsp) | $(convertpagebreak) > $@

doc/CHANGELOG.md: tmp/changelog.md
	$(convertnbsp) $+ | $(convertpagebreak) > $@

########################################################################
### pdf
########################################################################

convertbsp0 = sed -e 's@\\\[@[@g'
convertbsp1 = sed -e 's@\\\]@]@g'
convertbsp2 = sed -e 's@\\_]@_@g'

tmp/readme.tex: src/readme.tex
	cp src/readme.tex tmp

tmp/%.tex: tmp/%.md tmp/links_local.md tmp/images_local_pdf.md
	$(PANDOC) --smart $+ -t latex --columns=72 --chapters -o $@

tmp/api.tex: tmp/api.md tmp/links_local.md tmp/images_local_pdf.md
	$(PANDOC) tmp/api.md --filter src/IncludeFilter.py -t markdown -o tmp/api_inc.md
	cat tmp/api_inc.md | $(convertbsp0) | $(convertbsp1) | $(convertbsp2) > tmp/api_inc1.md
	$(PANDOC) tmp/api_inc1.md tmp/links_local.md tmp/images_local_pdf.md -f markdown --indented-code-classes=cpp --smart -t latex --columns=72 --chapters -o $@

doc/readme.pdf: preamble.tex tmp/version

doc/readme.pdf: tmp/readme.tex tmp/overview.tex tmp/changelog.tex tmp/platforms.tex tmp/support.tex tmp/acknowledgement.tex tmp/install.tex tmp/compilation.tex tmp/api.tex tmp/tutorials.tex $(images_jpg) $(images_pdf)
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
	rm -rf doc tmp man www
