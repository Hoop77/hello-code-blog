TRACKING_CODE = 'UA-131477483-1';
ENABLE_COOKIE_USAGE = 'enable-cookie-usage';
cookieUsageEnabled = false;

function loadGoogleAnalytics() {
    $.getScript('https://www.googletagmanager.com/gtag/js?id=' + TRACKING_CODE, function() {
        window.dataLayer = window.dataLayer || [];
        function gtag() {
            dataLayer.push(arguments); 
        }
        gtag('js', new Date());
        gtag('config', TRACKING_CODE);
    });
}

function setCookie(cname, cvalue, exdays) {
    var d = new Date();
    d.setTime(d.getTime() + (exdays*24*60*60*1000));
    var expires = "expires="+ d.toUTCString();
    document.cookie = cname + "=" + cvalue + ";" + expires + ";path=/";
}

function getCookie(cname) {
    var name = cname + "=";
    var decodedCookie = decodeURIComponent(document.cookie);
    var ca = decodedCookie.split(';');
    for (var i = 0; i < ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0) == ' ') {
            c = c.substring(1);
        }
        if (c.indexOf(name) == 0) {
            return c.substring(name.length, c.length);
        }
    }
    return "";
}

function showCookieConsent() {
    $("body").append(COOKIE_CONSENT);
}

function hideCookieConsent() {
    $(".cookie-consent").remove();
}

function enableCookieUsage() {
    setCookie(ENABLE_COOKIE_USAGE, 'true', 365);
    cookieUsageEnabled = true;
    $('#googleAnalyticsCheckbox').prop('checked', true);
}

function disableCookieUsage() {
    setCookie(ENABLE_COOKIE_USAGE, 'false', 365);
    cookieUsageEnabled = false;
    $('#googleAnalyticsCheckbox').prop('checked', false);
}

function checkCookies() {
    if (typeof IGNORE_COOKIE_CONSENT !== 'undefined' && IGNORE_COOKIE_CONSENT) {
        return;
    }

    cookieValue = getCookie(ENABLE_COOKIE_USAGE);
    if (cookieValue != 'true' && cookieValue != 'false') {
        showCookieConsent();
        return;
    }

    if (cookieValue == 'true') {
        loadGoogleAnalytics();
        cookieUsageEnabled = true;
    }
}

$(document).ready(function() {
    checkCookies();

    if (cookieUsageEnabled) {
        $('#googleAnalyticsCheckbox').prop('checked', true);
    }
    else {
        $('#googleAnalyticsCheckbox').prop('checked', false);
    }

    // Enable/disable cookie usage.
    $('#googleAnalyticsCheckbox').click(function(){
        if ($(this).prop('checked') == false){
            disableCookieUsage();
        }
        else {
            enableCookieUsage();
        }
    });
});