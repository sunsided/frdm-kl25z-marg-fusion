namespace WindowsFormsApplication1
{
    /// <summary>
    /// Struct Mode
    /// </summary>
    public struct Mode
    {
        /// <summary>
        /// The name
        /// </summary>
        private readonly string _name;

        /// <summary>
        /// The value
        /// </summary>
        private readonly byte _value;

        /// <summary>
        /// Initializes a new instance of the <see cref="Mode"/> struct.
        /// </summary>
        /// <param name="name">The name.</param>
        /// <param name="value">The value.</param>
        public Mode(string name, byte value)
        {
            _name = name;
            _value = value;
        }

        /// <summary>
        /// The name
        /// </summary>
        public string Name
        {
            get { return _name; }
        }

        /// <summary>
        /// The value
        /// </summary>
        public byte Value
        {
            get { return _value; }
        }
    }
}
