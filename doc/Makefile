webpages =  www/index.html www/api.html www/tutorials.html www/downloads.html www/renderer.html 

all: doc www
www: $(webpages)
doc: ../README.md ../readme.pdf
bin: README_bin.md readme_bin.pdf 
images: images/tutorial00.jpg images/tutorial01.jpg images/tutorial02.jpg images/tutorial03.jpg images/tutorial04.jpg images/tutorial05.jpg images/tutorial06.jpg images/tutorial07.jpg

.PHONY: all www doc images bin

replace=sed \
-e '/<CONTENT_OVERVIEW>/{r src/overview.html' -e 'd' -e '}' \
-e '/<CONTENT_NEW_FEATURES>/{r src/new_features.html' -e 'd' -e '};' \
-e '/<CONTENT_SUPPORT>/{r src/support.html' -e 'd' -e '};' \
-e '/<CONTENT_PLATFORMS>/{r src/platforms.html' -e 'd' -e '};' \
-e '/<CONTENT_DOWNLOADING>/{r src/downloading.html' -e 'd' -e '};' \
-e '/<CONTENT_COMPILATION>/{r src/compilation.html' -e 'd' -e '};' \
-e '/<CONTENT_INSTALL>/{r src/install.html' -e 'd' -e '};' \
-e '/<CONTENT_API>/{r src/api.html' -e 'd' -e '};' \
-e '/<CONTENT_TUTORIALS>/{r src/tutorials.html' -e 'd' -e '};' \
-e '/<CONTENT_RENDERER>/{r src/renderer.html' -e 'd' -e '};' \
-e '/<EMBREE_VERSION>/{r version' -e 'd' -e '};' \
$(1) > $(2)

replacelinks=sed -i\
-e 's/href="api.html"/href="\#api"/' \
-e 's@href="renderer.html"@href="http://embree.github.io/renderer.html"@' \
$(1)

version: ../build/version.h
	sed ':a;N;s/.*\"\(.*\)\".*/\1/;ba' $< > $@


### webpages
########################################################################

markdown2web = pandoc --email-obfuscation=none -f markdown $(filter-out src/webtemplate.html,$+) --template src/webtemplate -V select_$(basename $(@F)) -o $@

$(webpages): src/links_web.md src/webtemplate.html
www/%.html:
	$(markdown2web)

www/index.html: src/overview.md src/new_features.html src/support.md
www/downloads.html: src/platforms.md src/downloading.html src/compilation.md
www/tutorials.html: src/tutorials.md
www/renderer.html: src/renderer.md
www/api.html: src/api.md
	$(markdown2web) --indented-code-classes=cpp 


### images
########################################################################

images/%.jpg: 
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -size 512 512 -o images/`basename -s .jpg $@`.tga
	convert images/`basename -s .jpg $@`.tga images/`basename -s .jpg $@`.jpg
	rm images/`basename -s .jpg $@`.tga

images/tutorial03.jpg: 
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -c ~/Work/models/embree/crown/crown_paper.ecs -size 512 512 -frames 0 16 -o images/`basename -s .jpg $@`.tga
	convert images/`basename -s .jpg $@`.tga images/`basename -s .jpg $@`.jpg
	rm images/`basename -s .jpg $@`.tga

images/tutorial06.jpg: 
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -c ~/Work/models/embree/crown/crown_paper.ecs -size 512 512 -frames 0 16 -o images/`basename -s .jpg $@`.tga
	convert images/`basename -s .jpg $@`.tga images/`basename -s .jpg $@`.jpg
	rm images/`basename -s .jpg $@`.tga

readme.html: version src/*.html
	$(call replace,src/readme.html,$@)
	$(call replacelinks,$@)

../readme.pdf: readme.html
	html2ps -t -f html2ps.conf readme.html > readme.ps
	sed -i -e 's/\\240\\240/ /' readme.ps
	ps2pdf readme.ps 
	cp readme.pdf ..


### markdown
########################################################################

metadata.md: src/metadata.md version
	sed -e "s/<EMBREE_VERSION>/`cat version`/" $< > $@

../README.md: metadata.md src/overview.md src/platforms.md src/support.md src/compilation.md src/api.md src/tutorials.md src/links_local.md
	cat $+ > $@

README_bin.md: metadata.md src/overview.md src/platforms.md src/support.md src/install.md src/api.md src/tutorials.md src/links_local.md
	cat $+ > $@


### pdf
########################################################################

markdown2tex = pandoc --columns=70 --chapters $+ -o $@

%.tex: src/%.md src/links_local.md
	$(markdown2tex)

api.tex: src/api.md src/links_local.md
	$(markdown2tex) --indented-code-classes=cpp
#	sed -i -e 's/\\texttt/\\path/g' $@

readme.pdf: readme.tex version overview.tex platforms.tex support.tex compilation.tex api.tex tutorials.tex
	xelatex $<
	xelatex $<
#	cp readme.pdf ..


readme_bin.html: version src/*.html
	$(call replace,src/readme_bin.html,$@)
	$(call replacelinks,$@)

readme_bin.pdf: readme_bin.html
	html2ps -t -f html2ps.conf readme_bin.html > readme_bin.ps
	sed -i -e 's/\\240\\240/ /' readme_bin.ps
	ps2pdf readme_bin.ps 


### clean
########################################################################

clean:
	rm -f version
	rm -f metadata.md
	rm -f readme.pdf
	rm -f readme.ps
	rm -f readme.html
	rm -f README_bin.md
	rm -f readme_bin.pdf
	rm -f readme_bin.ps
	rm -f readme_bin.html
	rm -f $(webpages)
#	rm images/*.jpg
