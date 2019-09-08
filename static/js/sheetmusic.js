$(document).ready(function()
{
    $('.sheetmusic').each(function(idx, el) {
        var file = $(el).data('file')
        var osmd = new opensheetmusicdisplay.OpenSheetMusicDisplay(el);
        osmd.setOptions({
            drawTitle: false,
            drawSubtitle: false,
            drawComposer: false,
            drawLyricist: false,
            drawPartNames: false,
            drawPartAbbreviations: false,
            drawFingering: false,
            drawCredits: false,
            drawHiddenNotes: false
        });
        osmd.load(file).then(function() { osmd.render(); });
    });
});