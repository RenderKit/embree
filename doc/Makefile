
all: images www/index.html www/api.html www/tutorials.html www/downloads.html www/renderer.html ../readme.txt ../readme.pdf 
images: images/tutorial00.jpg images/tutorial01.jpg images/tutorial02.jpg images/tutorial03.jpg images/tutorial04.jpg images/tutorial05.jpg images/tutorial06.jpg images/tutorial07.jpg

replace=sed \
-e '/<CONTENT_OVERVIEW>/{r src/overview.html' -e 'd' -e '}' \
-e '/<CONTENT_NEW_FEATURES>/{r src/new_features.html' -e 'd' -e '};' \
-e '/<CONTENT_SUPPORT>/{r src/support.html' -e 'd' -e '};' \
-e '/<CONTENT_PLATFORMS>/{r src/platforms.html' -e 'd' -e '};' \
-e '/<CONTENT_DOWNLOADING>/{r src/downloading.html' -e 'd' -e '};' \
-e '/<CONTENT_COMPILATION>/{r src/compilation.html' -e 'd' -e '};' \
-e '/<CONTENT_API>/{r src/api.html' -e 'd' -e '};' \
-e '/<CONTENT_TUTORIALS>/{r src/tutorials.html' -e 'd' -e '};' \
-e '/<CONTENT_RENDERER>/{r src/renderer.html' -e 'd' -e '};' \
-e '/<EMBREE_VERSION>/{r version' -e 'd' -e '};' \
$(1) > $(2)

version: ../kernels/version.h
	sed 's/.*\"\(.*\)\".*/\1/g' $< > $@

www/index.html: version src/*.html src/www/index.html
	$(call replace,src/www/index.html,www/index.html)

www/api.html: version src/*.html src/www/api.html
	$(call replace,src/www/api.html,www/api.html)

www/tutorials.html: version src/*.html src/www/tutorials.html
	$(call replace,src/www/tutorials.html,www/tutorials.html)

www/downloads.html: version src/*.html src/www/downloads.html
	$(call replace,src/www/downloads.html,www/downloads.html)

www/renderer.html: version src/*.html src/www/renderer.html
	$(call replace,src/www/renderer.html,www/renderer.html)

images/%.jpg: 
	echo `basename -s .jpg $@`
	../build/`basename -s .jpg $@` -size 512 512 -frames 0 16 -o images/`basename -s .jpg $@`.tga
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
	$(call replace,src/readme.html,readme.html)

../readme.txt: readme.html
	html2text -rcfile html2text.conf readme.html > readme.txt
	cp readme.txt ..

../readme.pdf: readme.html
	html2ps -t -f html2ps.conf readme.html > readme.ps
	ps2pdf readme.ps 
	cp readme.pdf ..

clean:
	rm version
	rm readme.txt
	rm readme.pdf
	rm readme.ps
	rm readme.html
	rm www/*.html
	rm images/*.jpg
