
all: readme.txt www/index.html www/api.html www/tutorials.html www/downloads.html www/renderer.html
#embree.pdf embree.ps embree.html 

#replace=sed -e '/<CONTENT_OVERVIEW>/{r src/overview.html' -e 'd};' -e '/<CONTENT_NEW_FEATURES>/{r src/new_features.html' -e 'd};' -e '/<CONTENT_PLATFORMS>/{r src/platforms.html' -e 'd};' -e '/<CONTENT_DOWNLOADING>/{r src/downloading.html' -e 'd};' -e '/<CONTENT_COMPILATION>/{r src/compilation.html' -e 'd};' -e '/<CONTENT_API>/{r src/api.html' -e 'd};' -e '/<CONTENT_TUTORIALS>/{r src/tutorials.html' -e 'd};' -e '/<CONTENT_RENDERER>/{r src/renderer.html' -e 'd};' $(1) > $(2)

replace=sed -e '/<CONTENT_OVERVIEW>/{r src/overview.html' -e 'd' -e '}' -e '/<CONTENT_NEW_FEATURES>/{r src/new_features.html' -e 'd' -e '};' -e '/<CONTENT_PLATFORMS>/{r src/platforms.html' -e 'd' -e '};' -e '/<CONTENT_DOWNLOADING>/{r src/downloading.html' -e 'd' -e '};' -e '/<CONTENT_COMPILATION>/{r src/compilation.html' -e 'd' -e '};' -e '/<CONTENT_API>/{r src/api.html' -e 'd' -e '};' -e '/<CONTENT_TUTORIALS>/{r src/tutorials.html' -e 'd' -e '};' -e '/<CONTENT_RENDERER>/{r src/renderer.html' -e 'd' -e '};' $(1) > $(2)

#add_linebreaks=sed -e 's|</h2>|</h2><br>|g;' -e 's|</h3>|</h3><br>|g;' -e 's|</h4>|</h4><br>|g;' -e 's|</p>|</p><br>|g;' $(1) > $(2)

www/index.html: src/*.html src/www/frame_index.html
	$(call replace,src/www/frame_index.html,www/index.html)

www/api.html: src/*.html src/www/frame_api.html
	$(call replace,src/www/frame_api.html,www/api.html)

www/tutorials.html: src/*.html src/www/frame_tutorials.html
	$(call replace,src/www/frame_tutorials.html,www/tutorials.html)

www/downloads.html: src/*.html src/www/frame_downloads.html
	$(call replace,src/www/frame_downloads.html,www/downloads.html)

www/renderer.html: src/*.html src/www/frame_renderer.html
	$(call replace,src/www/frame_renderer.html,www/renderer.html)

embree.html: src/*.html
	$(call replace,src/frame.html,embree.html)

embree.ps: embree.html
	html2ps embree.html > embree.ps

embree.pdf: embree.ps
	ps2pdf embree.ps 

readme.txt: src/*.html
#	$(call add_linebreaks,src/overview.html,overview.html)
	$(call replace,src/readme.txt,readme.html)
	html2text readme.html -o readme.txt
	rm readme.html

clean:
	rm readme.txt
	rm embree.pdf
	rm embree.ps
	rm embree.html
	rm www/*.html
