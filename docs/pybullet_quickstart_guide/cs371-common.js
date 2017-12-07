
// Common code for styling all CS371 web pages

if (location.href.indexOf('?') == -1) {
    // Force a reload by making a bogus query.
    var i = ((location.href.indexOf('#') + 1) || (location.href.length + 1)) - 1;
    location = location.href.substring(0, i) + '?' + location.href.substring(i);
}

markdeepOptions = {tocStyle: 'long'};

document.write("<link href='https://fonts.googleapis.com/css?family=Roboto:400,300,100,100italic,300italic,400italic,700' rel='stylesheet' type='text/css'>" +
               "<link rel='stylesheet' type='text/css' href='cs371.css'>");

// Conceal list items that use a '+', except when viewed locally
if (/^file:\/{3}/i.test(window.location.href)) {
    document.write('<style>li.plus { color: #D88; }</style>');
} else {
    document.write('<style>li.plus { display: none; }</style>');
}

document.write('<!-- Markdeep: --><style class="fallback">body{visibility:hidden;white-space:pre;font-family:monospace}</style><script src="markdeep.js"></script><script>window.alreadyProcessedMarkdeep||(document.body.style.visibility="visible")</script>');


