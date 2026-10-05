using System.Globalization;
using System.Text.RegularExpressions;
using System.Windows;

namespace TwoLinkCommissioning;

// A user-drawn contour: either a list of points ("x y" or "x,y", one per line or separated by ';') or one SVG path
// ("M L H V C S Q T Z", absolute or relative).  Coordinates are millimetres relative to the shape's anchor (x, y);
// SVG's y axis points down, so it is flipped.  The machine has no pen lift, so exactly one contour is accepted.
internal static class CustomPath
{
    internal const int MaxPoints = 5000;
    // A turn sharper than this between flattened curve pieces is a real corner: the pen stops there like a rectangle corner.
    private const double CornerDegrees = 25;
    private static readonly Regex Token = new(@"([MmLlHhVvCcSsQqTtAaZz])|([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)", RegexOptions.CultureInvariant);

    public sealed record Result(List<Vector> Points, List<bool> Corner, bool Closed)
    {
        // True when every vertex is a stop, so the contour is plain straight strokes.
        public bool AllCorners => Corner.All(c => c);
    }

    public static Result Parse(string? text)
    {
        text = (text ?? "").Trim();
        if (text.Length == 0) throw Lang.Error("err.customEmpty");
        var result = char.IsLetter(text[0]) ? ParseSvg(text) : ParsePoints(text);
        return Clean(result);
    }

    private static Result ParsePoints(string text)
    {
        var points = new List<Vector>();
        int line = 0;
        foreach (string row in text.Split(['\n', ';'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            line++;
            var numbers = Token.Matches(row).Where(m => m.Groups[2].Success).Select(m => m.Groups[2].Value).ToArray();
            bool onlyNumbers = Token.Replace(row, "").Replace(",", "").Trim().Length == 0 && Token.Matches(row).All(m => m.Groups[2].Success);
            if (numbers.Length != 2 || !onlyNumbers) throw Lang.Error("err.customPointLine", line);
            points.Add(new Vector(Number(numbers[0]), Number(numbers[1])));
            if (points.Count > MaxPoints) throw Lang.Error("err.tooManyPoints");
        }
        return new Result(points, points.Select(_ => true).ToList(), false);
    }

    private static double Number(string text) =>
        double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) && double.IsFinite(v) && Math.Abs(v) <= 20000
            ? v : throw Lang.Error("err.customNumber", text);

    private static Result ParseSvg(string text)
    {
        var tokens = Token.Matches(text).Select(m => m.Groups[1].Success ? (Command: m.Groups[1].Value[0], Value: 0.0) : (Command: '\0', Value: Number(m.Groups[2].Value))).ToList();
        if (Token.Replace(text, "").Replace(",", "").Trim().Length != 0) throw Lang.Error("err.customSyntax");
        int at = 0;
        var points = new List<Vector>(); var corner = new List<bool>();
        Vector current = default, start = default, lastControl = default; char previous = ' ';
        bool drawn = false, closed = false, haveStart = false;
        double Next() => at < tokens.Count && tokens[at].Command == '\0' ? tokens[at++].Value : throw Lang.Error("err.customSyntax");
        bool MoreNumbers() => at < tokens.Count && tokens[at].Command == '\0';
        void Add(Vector p, bool isCorner) {
            if (points.Count >= MaxPoints) throw Lang.Error("err.tooManyPoints");
            points.Add(p); corner.Add(isCorner);
        }
        void Curve(Func<double, Vector> at, double controlLength)
        {
            int n = Math.Clamp((int)Math.Ceiling(controlLength), 4, 400);
            for (int i = 1; i < n; i++) Add(at((double)i / n), false);
        }
        while (at < tokens.Count)
        {
            char command = tokens[at].Command;
            if (command == '\0') throw Lang.Error("err.customSyntax");
            at++;
            bool relative = char.IsLower(command); char kind = char.ToUpperInvariant(command);
            if (kind == 'A') throw Lang.Error("err.customCommand", command);
            if (kind == 'Z')
            {
                if (!haveStart) throw Lang.Error("err.customSyntax");
                if ((current - start).Length > 1e-9) Add(start, true);
                current = start; closed = true; previous = 'Z'; continue;
            }
            if (closed) throw Lang.Error("err.customContours");   // anything after Z starts another contour
            if (kind == 'M')
            {
                if (haveStart) throw Lang.Error("err.customContours");
            }
            else if (!haveStart) throw Lang.Error("err.customSyntax");
            do
            {
                switch (kind)
                {
                    case 'M':
                    {
                        double x = Next(), y = Next();
                        var p = relative && haveStart ? current + new Vector(x, -y) : new Vector(x, -y);
                        if (!haveStart) { start = current = p; haveStart = true; Add(p, true); kind = 'L'; }   // further pairs after the first moveto are linetos
                        break;
                    }
                    case 'L': { double x = Next(), y = Next(); current = relative ? current + new Vector(x, -y) : new Vector(x, -y); Add(current, true); drawn = true; break; }
                    case 'H': { double x = Next(); current = new Vector(relative ? current.X + x : x, current.Y); Add(current, true); drawn = true; break; }
                    case 'V': { double y = Next(); current = new Vector(current.X, relative ? current.Y - y : -y); Add(current, true); drawn = true; break; }
                    case 'C':
                    case 'S':
                    {
                        Vector c1;
                        if (kind == 'C') { double x1 = Next(), y1 = Next(); c1 = relative ? current + new Vector(x1, -y1) : new Vector(x1, -y1); }
                        else c1 = previous is 'C' or 'S' ? current * 2 - lastControl : current;
                        double x2 = Next(), y2 = Next(), x = Next(), y = Next();
                        Vector c2 = relative ? current + new Vector(x2, -y2) : new Vector(x2, -y2), end = relative ? current + new Vector(x, -y) : new Vector(x, -y);
                        Vector p0 = current;
                        Curve(t => { double u = 1 - t; return p0 * (u * u * u) + c1 * (3 * u * u * t) + c2 * (3 * u * t * t) + end * (t * t * t); },
                            (c1 - p0).Length + (c2 - c1).Length + (end - c2).Length);
                        Add(end, false); current = end; lastControl = c2; drawn = true; previous = 'C'; continue;
                    }
                    case 'Q':
                    case 'T':
                    {
                        Vector c;
                        if (kind == 'Q') { double x1 = Next(), y1 = Next(); c = relative ? current + new Vector(x1, -y1) : new Vector(x1, -y1); }
                        else c = previous is 'Q' or 'T' ? current * 2 - lastControl : current;
                        double x = Next(), y = Next();
                        Vector end = relative ? current + new Vector(x, -y) : new Vector(x, -y), p0 = current;
                        Curve(t => { double u = 1 - t; return p0 * (u * u) + c * (2 * u * t) + end * (t * t); }, (c - p0).Length + (end - c).Length);
                        Add(end, false); current = end; lastControl = c; drawn = true; previous = 'Q'; continue;
                    }
                }
                previous = kind;
            } while (MoreNumbers());
        }
        if (!drawn) throw Lang.Error("err.customEmpty");
        return new Result(points, corner, closed);
    }

    // Drop repeated points, and decide which curve joints are corners (the others are smooth: no stop there).
    private static Result Clean(Result raw)
    {
        var points = new List<Vector>(); var corner = new List<bool>();
        for (int i = 0; i < raw.Points.Count; i++)
        {
            if (points.Count > 0 && (raw.Points[i] - points[^1]).Length < 1e-9) { corner[^1] |= raw.Corner[i]; continue; }
            points.Add(raw.Points[i]); corner.Add(raw.Corner[i]);
        }
        if (points.Count < 2 || Enumerable.Range(1, points.Count - 1).Sum(i => (points[i] - points[i - 1]).Length) < 0.001) throw Lang.Error("err.customEmpty");
        corner[0] = corner[^1] = true;
        for (int i = 1; i < points.Count - 1; i++)
        {
            if (corner[i]) continue;
            // Flattened curve interior: a vertex only becomes a stop when the contour really turns there.
            Vector before = points[i] - points[i - 1], after = points[i + 1] - points[i];
            double turn = Math.Abs(Vector.AngleBetween(before, after));
            corner[i] = turn > CornerDegrees;
        }
        return new Result(points, corner, raw.Closed);
    }

    public static void SelfTest()
    {
        static void Check(bool value, string message) { if (!value) throw new InvalidOperationException("Custom path: " + message); }
        static void Reject(string text, string message) {
            try { Parse(text); } catch (ArgumentException) { return; }
            throw new InvalidOperationException("Custom path accepted: " + message);
        }
        var square = Parse("0 0\n10,0 ; 10 10\n0 10\n0 0");
        Check(square.Points.Count == 5 && square.AllCorners && !square.Closed && square.Points[2] == new Vector(10, 10), "point list");
        var triangle = Parse("M 0 0 L 10 0 L 10 10 Z");   // SVG y points down: flipped
        Check(triangle.Closed && triangle.Points.Count == 4 && triangle.Points[2] == new Vector(10, -10) && triangle.Points[3] == triangle.Points[0] && triangle.AllCorners, "svg lines + close");
        var relative = Parse("m 5 5 h 10 v 10 h -10 z");
        Check(relative.Points.Count == 5 && relative.Points[1] == new Vector(15, -5) && relative.Points[2] == new Vector(15, -15) && relative.Points[3] == new Vector(5, -15) && relative.Closed, "relative commands");
        var curve = Parse("M 0 0 C 0 -20 20 -20 20 0 L 40 0");
        Check(!curve.AllCorners && curve.Points.Count > 10 && curve.Corner[0] && curve.Corner[^1] && curve.Corner.Count(c => c) >= 3, "cubic is flattened, joint with the line is a corner");
        Check(curve.Points.Zip(curve.Points.Skip(1)).SkipLast(1).All(p => (p.Second - p.First).Length < 2.5), "flattened pieces are short (the straight tail is one stroke)");
        var quad = Parse("M0,0 Q 10,-20 20,0 T 40,0");
        Check(quad.Points.Count > 8 && quad.Points[^1] == new Vector(40, 0), "quadratic and smooth quadratic");
        Check(Parse("M 0 0 L 10 0 10 10").Points.Count == 3, "implicit lineto repetition");
        Reject("", "empty"); Reject("M 0 0", "single point"); Reject("0 0\n1", "bad point line"); Reject("0 0\n1 x", "letters in a point line");
        Reject("M 0 0 L 10 0 M 20 0 L 30 0", "two contours need a pen lift"); Reject("M 0 0 L 10 0 Z L 5 5", "drawing after close");
        Reject("M 0 0 A 5 5 0 0 1 10 0", "arc command"); Reject("L 10 0", "no moveto"); Reject("M 0 0 L 10", "missing coordinate");
        Reject("M 0 0 L 1e9 0", "huge coordinate"); Reject("M 0 0 L 5 0 # junk", "junk");
    }
}
